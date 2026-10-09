"""S17.9 ROS-only bridge/client: low-rate references and snapshots, no MuJoCo import."""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import socket
import sys
import threading
import time

import rclpy
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import SetBool
from s17_interfaces.msg import SurfaceReference, SurfaceState
from s17_interfaces.action import FollowSurface

TOPIC, STATE, ACTION, PAUSE = '/s17_9/reference', '/s17_9/state', '/s17_9/follow_surface', '/s17_9/pause'


def rpc(path, packet):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as conn:
        conn.settimeout(3.); conn.connect(str(path))
        with conn.makefile('rwb') as stream:
            stream.write((json.dumps(packet, allow_nan=False)+'\n').encode()); stream.flush()
            result = json.loads(stream.readline(65536))
    if not result['ok']: raise ValueError(result['reason'])
    return result['value']


def state_msg(state):
    msg = SurfaceState()
    for name in msg.get_fields_and_field_types(): setattr(msg, name, state[name])
    return msg


class Bridge(Node):
    def __init__(self, args):
        super().__init__('s17_9_bridge')
        self.args, self.lock, self.busy = args, threading.Lock(), False
        self.reference, self.latest = None, None
        self.sequence, self.pending = 0, deque(maxlen=32)
        group = ReentrantCallbackGroup()
        self.ref_pub = self.create_publisher(SurfaceReference, TOPIC, 1)
        self.state_pub = self.create_publisher(SurfaceState, STATE, 1)
        self.sub = self.create_subscription(SurfaceReference, TOPIC, self.receive, 1, callback_group=group)
        self.snapshot_timer = self.create_timer(.05, self.publish_state, callback_group=group)
        # Default mutually-exclusive callback group keeps reference deque/sequence serialized.
        self.command_timer = self.create_timer(args.command_period, self.send_reference)
        self.pause_service = self.create_service(SetBool, PAUSE, self.pause, callback_group=group)
        self.action = ActionServer(self, FollowSurface, ACTION, execute_callback=self.execute,
                                  goal_callback=self.accept, cancel_callback=lambda _: CancelResponse.ACCEPT,
                                  callback_group=group)

    def receive(self, msg):
        packet = {name: getattr(msg, name) for name in msg.get_fields_and_field_types()}
        try: rpc(self.args.socket, dict(op='reference', **packet))
        except Exception as error: self.get_logger().error(f'reference IPC failed: {error}')

    def publish_state(self):
        try:
            state = rpc(self.args.socket, dict(op='state'))  # Snapshot read does not advance physics.
            with self.lock: self.latest = state
            self.state_pub.publish(state_msg(state))
        except Exception as error:
            self.get_logger().error(f'state unavailable: {error}')

    def send_reference(self):
        now = time.monotonic()
        with self.lock:
            reference, state = self.reference, self.latest
        if reference is not None and (self.args.stop_after < 0 or state is None or state['task_elapsed_s'] < self.args.stop_after):
            msg = SurfaceReference(); self.sequence += 1
            msg.generation, msg.sequence, msg.issued_monotonic_s = reference['generation'], self.sequence, now
            msg.distance_m, msg.leg_duration_s, msg.normal_target_n = reference['distance_m'], reference['leg_duration_s'], 2.
            self.pending.append((now+self.args.command_delay, msg))  # Real bounded wall delay injection.
        while self.pending and self.pending[0][0] <= now:
            _, msg = self.pending.popleft()
            self.ref_pub.publish(msg)

    def pause(self, request, response):
        try:
            state = rpc(self.args.socket, dict(op='pause', paused=request.data))
            response.success, response.message = True, json.dumps(state)
        except Exception as error:
            response.success, response.message = False, str(error)
        return response

    def accept(self, goal):
        with self.lock:
            valid = (all(math.isfinite(v) for v in (goal.distance_m, goal.leg_duration_s, goal.command_timeout_s))
                     and .01 <= goal.distance_m <= .03 and 2 <= goal.leg_duration_s <= 3 and .25 <= goal.command_timeout_s <= 1)
            if self.busy or not valid: return GoalResponse.REJECT
            self.busy = True
        return GoalResponse.ACCEPT

    def execute(self, handle):
        result = FollowSurface.Result()
        generation = None
        try:
            goal = handle.request
            state = rpc(self.args.socket, dict(op='begin', distance_m=goal.distance_m,
                        leg_duration_s=goal.leg_duration_s, command_timeout_s=goal.command_timeout_s))
            generation = state['generation']
            with self.lock:
                self.reference = dict(generation=generation, distance_m=goal.distance_m, leg_duration_s=goal.leg_duration_s)
            while True:
                state = rpc(self.args.socket, dict(op='state'))
                if time.monotonic()-state['snapshot_monotonic_s'] > .5:
                    raise TimeoutError('worker snapshot stale')
                if state['status'] == 'active' and handle.is_cancel_requested:
                    state = rpc(self.args.socket, dict(op='cancel', generation=generation))
                feedback = FollowSurface.Feedback(); feedback.state = state_msg(state)
                handle.publish_feedback(feedback)
                if state['status'] in ('succeeded', 'failed', 'canceled'):
                    result.success = state['status'] == 'succeeded'
                    result.reason = state['reason']; result.final_state = state_msg(state)
                    if state['status'] == 'canceled': handle.canceled()
                    elif result.success: handle.succeed()
                    else: handle.abort()
                    self.get_logger().info(f'terminal={state["status"]}; {state["reason"]}')
                    return result
                time.sleep(.05)  # Only feedback pacing; no tick/request grants to the controller.
        except Exception as error:
            # Best-effort cancel; if IPC fails, the worker's wall lease independently expires.
            try: rpc(self.args.socket, dict(op='cancel', generation=generation))
            except Exception: pass
            if handle.is_active: handle.abort()
            result.reason = f'{error}; inspect worker state; IPC failure may leave state uncertain'
            return result
        finally:
            with self.lock: self.reference, self.busy = None, False


def wait(node, future, timeout):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done(): raise TimeoutError('ROS future timeout; remote outcome uncertain')
    return future.result()


def run_client(node, args):
    feedback = []
    # Keep bounded history; these are actual ROS state messages, separate from Action feedback.
    states = deque(maxlen=400)
    sub = node.create_subscription(SurfaceState, STATE, lambda msg: states.append(msg), 1)
    client = ActionClient(node, FollowSurface, ACTION)
    if not client.wait_for_server(timeout_sec=5): print('action unavailable'); return 3
    goal = FollowSurface.Goal(); goal.distance_m, goal.leg_duration_s, goal.command_timeout_s = .03, 2., .5
    handle = wait(node, client.send_goal_async(goal, feedback_callback=lambda msg: feedback.append(msg.feedback.state)), 3)
    if not handle.accepted: print('goal rejected'); return 2
    future, start, cancel = handle.get_result_async(), time.monotonic(), None
    while not future.done() and time.monotonic()-start < 35:
        rclpy.spin_once(node, timeout_sec=.02)
        if cancel is None and args.cancel_at >= 0 and feedback and feedback[-1].task_elapsed_s >= args.cancel_at:
            cancel = handle.cancel_goal_async()
    if not future.done():
        handle.cancel_goal_async(); print('result timeout; requested cancel, remote state uncertain'); return 4
    wrapped = future.result()
    acknowledged = bool(wait(node, cancel, 2).goals_canceling) if cancel is not None else False
    final = wrapped.result.final_state
    # Terminal task must not freeze physics; inspect two fresh worker snapshots.
    before = rpc(args.socket, dict(op='state')); time.sleep(.25); after = rpc(args.socket, dict(op='state'))
    assert after['sim_time_s'] > before['sim_time_s']+.05 and not after['paused']
    pause_check = None
    if args.pause_check:
        service = node.create_client(SetBool, PAUSE)
        assert service.wait_for_service(timeout_sec=3)
        request = SetBool.Request(); request.data = True
        paused = wait(node, service.call_async(request), 3); assert paused.success, paused.message
        a = rpc(args.socket, dict(op='state')); time.sleep(.2); b = rpc(args.socket, dict(op='state'))
        assert a['paused'] and b['paused'] and a['sim_time_s'] == b['sim_time_s'] and a['q_rad'] == b['q_rad'] and a['qvel_rad_s'] == b['qvel_rad_s']
        request.data = False; resumed = wait(node, service.call_async(request), 3); assert resumed.success
        time.sleep(.2); c = rpc(args.socket, dict(op='state')); assert c['sim_time_s'] > b['sim_time_s']+.05
        pause_check = dict(paused_before=a, paused_after=b, resumed=c)
    expected_status = 5 if args.cancel_at >= 0 else 6 if 0 <= args.stop_after <= 10 or args.command_delay >= .5 else 4
    assert wrapped.status == expected_status, (wrapped.status, wrapped.result.reason)
    assert feedback and states and all(time.monotonic()-s.snapshot_monotonic_s >= 0 for s in states)
    assert states[-1].generation == final.generation and len(states) >= 2
    if expected_status == 6: assert final.reason == 'command_timeout'
    if expected_status == 5: assert acknowledged and final.reason == 'cancel_applied'
    report = dict(status={4:'SUCCEEDED',5:'CANCELED',6:'ABORTED'}[wrapped.status], reason=wrapped.result.reason,
                  feedback_count=len(feedback), state_message_count=len(states), cancel_acknowledged=acknowledged,
                  terminal_state={name:(getattr(final,name).tolist() if hasattr(getattr(final,name), 'tolist') else getattr(final,name)) for name in final.get_fields_and_field_types()},
                  continuing_before=before, continuing_after=after, pause_check=pause_check, engineering_checks_passed=True)
    (args.output/'client_result.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2)); return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('role', choices=('server','client'))
    parser.add_argument('--socket', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--command-period', type=float, default=.1)
    parser.add_argument('--command-delay', type=float, choices=(0., .2, .7), default=0.)
    parser.add_argument('--stop-after', type=float, default=-1.)
    parser.add_argument('--cancel-at', type=float, default=-1.)
    parser.add_argument('--pause-check', action='store_true')
    args = parser.parse_args()
    if not (all(math.isfinite(v) for v in (args.command_period,args.command_delay,args.stop_after,args.cancel_at))
            and .05 <= args.command_period <= .2 and 0 <= args.command_delay <= 1
            and (args.stop_after == -1 or 0 <= args.stop_after <= 10) and (args.cancel_at == -1 or 0 <= args.cancel_at <= 10)):
        parser.error('bounded finite period/delay and nonnegative stop/cancel times (or -1) required')
    if os.environ.get('CONDA_PREFIX') or os.environ.get('CONDA_DEFAULT_ENV') or os.environ.get('ROS_DISTRO') != 'jazzy' or Path(sys.executable).resolve() != Path('/usr/bin/python3').resolve():
        raise RuntimeError('ROS-only system Python/Jazzy required')
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init(); node = None; executor = None
    try:
        if args.role == 'server':
            node = Bridge(args); executor = MultiThreadedExecutor(num_threads=4); executor.add_node(node); executor.spin()
        else:
            node = Node('s17_9_client'); return run_client(node,args)
    except KeyboardInterrupt: return 130
    finally:
        if executor is not None: executor.shutdown()
        if node is not None:
            if isinstance(node,Bridge): node.action.destroy()
            node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': raise SystemExit(main())
