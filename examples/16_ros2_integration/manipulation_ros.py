"""ROS-only S15.6b: typed planning + full manipulation action + sim clock/state."""
import argparse
import json
import math
from pathlib import Path
import socket
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient,ActionServer,GoalResponse,CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock as RosClock
from rclpy.clock import ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from tf2_ros import Buffer,TransformListener,TransformException
from s15_interfaces.srv import PlanManipulation
from s15_interfaces.action import ExecuteManipulation
from joint_states_tf import matrix

TOPIC='/s15_6b/object_pose';SERVICE='/s15_6b/plan';ACTION='/s15_6b/manipulate'
JOINTS=['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']


def rpc(path,message):
    with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as connection:
        connection.settimeout(20.)
        connection.connect(str(path))
        with connection.makefile('rwb') as stream:
            stream.write((json.dumps(message,allow_nan=False)+'\n').encode());stream.flush()
            response=json.loads(stream.readline(4_000_000))
    if not response['ok']:raise ValueError(response['reason'])
    return response['value']


def seconds(stamp):return stamp.sec+stamp.nanosec*1e-9


def sim_node(name):
    return Node(name,parameter_overrides=[Parameter('use_sim_time',value=True)])


class Perception(Node):
    def __init__(self,args):
        super().__init__('s15_6b_perception',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.args=args;self.publisher=self.create_publisher(PoseStamped,TOPIC,10)
        # The image is captured once at simulation t=0. Re-publishing does NOT refresh its timestamp.
        self.observation=rpc(args.socket,dict(op='observe'))
        self.timer=self.create_timer(.1,self.publish,clock=RosClock(clock_type=ClockType.STEADY_TIME))

    def publish(self):
        msg=PoseStamped();msg.header.frame_id='world';msg.header.stamp=Time(seconds=0).to_msg()
        msg.pose.position.x,msg.pose.position.y,msg.pose.position.z=self.observation['position']
        msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w=self.observation['quaternion_xyzw']
        self.publisher.publish(msg)


class Server(Node):
    def __init__(self,args):
        super().__init__('s15_6b_server',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.args=args;self.lock=threading.Lock();self.plan_id=None;self.busy=False
        self.clock_publisher=self.create_publisher(Clock,'/clock',10)
        self.joints=self.create_publisher(JointState,'/joint_states',10)
        group=ReentrantCallbackGroup()
        self.service=self.create_service(PlanManipulation,SERVICE,self.plan,callback_group=group)
        self.action=ActionServer(self,ExecuteManipulation,ACTION,execute_callback=self.execute,
                                goal_callback=self.accept,cancel_callback=self.cancel,callback_group=group)
        self.clock_publisher.publish(Clock(clock=Time(seconds=0).to_msg()))

    def plan(self,request,response):
        with self.lock:
            if self.busy:response.reason='execution_busy';return response
            self.plan_id=None
            try:
                msg=request.object_pose;p=msg.pose.position;q=msg.pose.orientation
                if msg.header.frame_id!='world':raise ValueError('expected_world_frame')
                age=seconds(self.get_clock().now().to_msg())-seconds(msg.header.stamp)
                if not 0<=age<=.5:raise ValueError('future_or_stale_capture_time')
                position=[p.x,p.y,p.z];quat=[q.x,q.y,q.z,q.w]
                if not all(math.isfinite(v) for v in (*position,*quat)):raise ValueError('nonfinite_pose')
                start=time.monotonic()
                planned=rpc(self.args.socket,dict(op='plan',position=position,quaternion=quat))
                if time.monotonic()-start>3:raise ValueError('short_planning_budget')
                self.plan_id=planned['plan_id']
                response.success=True;response.plan_id=self.plan_id;response.reason='initial_grasp/approach_plan_ready'
                self.get_logger().info(f'plan_ready: {self.plan_id}; candidate={planned["candidate_id"]}')
            except Exception as error:response.reason=str(error);self.get_logger().info(f'plan_rejected: {error}')
        return response

    def accept(self,goal):
        with self.lock:
            if self.busy or self.plan_id is None or goal.plan_id!=self.plan_id:return GoalResponse.REJECT
            self.busy=True
        return GoalResponse.ACCEPT

    def cancel(self,handle):return CancelResponse.ACCEPT

    def publish_state(self,state):
        stamp=Time(nanoseconds=round(state['simulation_time_s']*1e9)).to_msg()
        self.clock_publisher.publish(Clock(clock=stamp))
        joints=JointState();joints.header.stamp=stamp;joints.name=JOINTS
        joints.position=state['actual_joint_positions'];joints.velocity=state['actual_joint_velocities']
        self.joints.publish(joints)  # effort unknown; leave empty, never pretend measured zero.

    def execute(self,handle):
        result=ExecuteManipulation.Result()
        try:
            rpc(self.args.socket,dict(op='begin',plan_id=handle.request.plan_id))
            while True:
                if handle.is_cancel_requested:
                    snapshot=rpc(self.args.socket,dict(op='stop'))
                    if snapshot['running']:raise RuntimeError('stop_not_confirmed')
                    terminal=snapshot.get('result',{}).get('status')
                    # Cancellation can race with completion/artifact saving; preserve the actual outcome.
                    if terminal=='placement_passed':
                        handle.succeed();result.success=True;result.reason='placement_passed_before_cancel'
                    elif terminal=='failed':
                        handle.abort();result.success=False;result.reason='failed_before_cancel'
                    else:
                        handle.canceled();result.success=False;result.reason='simulation_paused_after_cancel'
                    result.diagnostics_json=json.dumps(snapshot)
                    self.get_logger().info(f'cancel_resolved: {result.reason}')
                    return result
                snapshot=rpc(self.args.socket,dict(op='tick'));state=snapshot['state']
                self.publish_state(state)
                feedback=ExecuteManipulation.Feedback();feedback.phase=state['phase']
                feedback.simulation_time_s=state['simulation_time_s'];feedback.actual_joint_positions=state['actual_joint_positions']
                handle.publish_feedback(feedback)
                if snapshot['done']:
                    passed=snapshot['result']['status']=='placement_passed'
                    if passed:handle.succeed()
                    else:handle.abort()
                    result.success=passed;result.reason=snapshot['result']['status'];result.diagnostics_json=json.dumps(snapshot)
                    self.get_logger().info(f'terminal={"SUCCEEDED" if passed else "ABORTED"}; phase={state["phase"]}')
                    return result
                time.sleep(.002)
        except Exception as error:
            try:snapshot=rpc(self.args.socket,dict(op='stop'));result.diagnostics_json=json.dumps(snapshot)
            except Exception:result.diagnostics_json=json.dumps(dict(remote_state='unknown'))
            if handle.is_active:handle.abort()
            result.success=False;result.reason=f'{type(error).__name__}: {error}'
            return result
        finally:
            with self.lock:self.busy=False;self.plan_id=None


def wait(node,future,timeout):
    rclpy.spin_until_future_complete(node,future,timeout_sec=timeout)
    if not future.done():raise TimeoutError('future_timeout; remote_state_unknown')
    return future.result()


def client(node,args):
    observations=[];feedbacks=[];joint_messages=[]
    subscription=node.create_subscription(PoseStamped,TOPIC,lambda msg:observations.append(msg),10)
    joint_subscription=node.create_subscription(JointState,'/joint_states',lambda msg:joint_messages.append(msg),10)
    buffer=Buffer(node=node);listener=TransformListener(buffer,node)
    deadline=time.monotonic()+8
    while not observations and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
    if not observations:print('status=perception_unavailable');return 3
    pose=observations[-1]
    if args.bad_frame:pose.header.frame_id='camera_optical'
    if args.stale:pose.header.stamp.sec-=2
    if args.bad_pose:pose.pose.position.x=10.
    if args.nonfinite:pose.pose.position.x=float('nan')
    planning=node.create_client(PlanManipulation,SERVICE)
    if not planning.wait_for_service(timeout_sec=3):print('status=service_unavailable');return 3
    req=PlanManipulation.Request();req.object_pose=pose
    response=wait(node,planning.call_async(req),5)
    print(f'planning_success={response.success}; reason={response.reason}',flush=True)
    if not response.success:return 1
    print(f'plan_id={response.plan_id}',flush=True)
    action=ActionClient(node,ExecuteManipulation,ACTION)
    if not action.wait_for_server(timeout_sec=3):return 3
    goal=ExecuteManipulation.Goal();goal.plan_id=response.plan_id
    def feedback(message):
        f=message.feedback;feedbacks.append(f)
        if len(feedbacks)==1 or f.phase!=feedbacks[-2].phase:
            print(f'phase={f.phase}; sim_time_s={f.simulation_time_s:.3f}',flush=True)
    handle=wait(node,action.send_goal_async(goal,feedback_callback=feedback),3)
    if not handle.accepted:print('execution_rejected=True');return 2
    print('execution_goal_accepted=True',flush=True)
    future=handle.get_result_async();start=time.monotonic();cancel_future=None;timeout_triggered=False
    while not future.done():
        rclpy.spin_once(node,timeout_sec=.02)
        elapsed=time.monotonic()-start
        phase_cancel=bool(args.cancel_phase and feedbacks and feedbacks[-1].phase==args.cancel_phase)
        time_cancel=args.cancel_after>=0 and elapsed>=args.cancel_after
        if cancel_future is None and (phase_cancel or time_cancel or elapsed>=args.execution_timeout):
            timeout_triggered=elapsed>=args.execution_timeout
            cancel_future=handle.cancel_goal_async()
            cancel_sent=time.monotonic()
            print(f'cancel_requested=True; reason={"timeout" if timeout_triggered else "user"}',flush=True)
        if cancel_future is not None and time.monotonic()-cancel_sent>25:
            print('status=cancel_terminal_timeout; remote_state_unknown');return 4
    if cancel_future is not None:
        ack=wait(node,cancel_future,2);print(f'cancel_acknowledged={bool(ack.goals_canceling)}')
    wrapped=future.result();snapshot=json.loads(wrapped.result.diagnostics_json)
    print(f'terminal_status={wrapped.status}; success={wrapped.result.success}; reason={wrapped.result.reason}; feedback_count={len(feedbacks)}')
    args.output.write_text(json.dumps(dict(terminal_status=wrapped.status,success=wrapped.result.success,reason=wrapped.result.reason,
        timeout_triggered=timeout_triggered,feedback_count=len(feedbacks),feedback_phases=list(dict.fromkeys(f.phase for f in feedbacks)),
        snapshot=snapshot),indent=2)+'\n')
    # Compare live RSP tool TF to MuJoCo scorer at the same simulation stamp.
    if snapshot.get('state',{}).get('tool_position_world_m') is not None:
        state=snapshot['state'];stamp=Time(nanoseconds=round(state['simulation_time_s']*1e9))
        end=time.monotonic()+3;tf=None
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
            try:tf=buffer.lookup_transform('world','tool',stamp);break
            except TransformException:pass
        if tf is None:
            print('live_tf_check=unavailable_at_terminal_stamp');return 5
        t=tf.transform.translation;p=state['tool_position_world_m']
        error=max(abs(a-b) for a,b in zip((t.x,t.y,t.z),p))
        actual_matrix=matrix(tf)
        rotation_error=max(abs(actual_matrix[i][j]-state['tool_rotation_world'][i][j]) for i in range(3) for j in range(3))
        print(f'live_tf_position_error_m={error:.3e}; live_tf_rotation_matrix_error={rotation_error:.3e}; joint_messages={len(joint_messages)}')
        if error>1e-8 or rotation_error>1e-8:return 5
    if timeout_triggered:return 4
    if args.cancel_phase or args.cancel_after>=0:return 0 if wrapped.status==GoalStatus.STATUS_CANCELED else 1
    return 0 if wrapped.status==GoalStatus.STATUS_SUCCEEDED and wrapped.result.success else 1


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('role',choices=('perception','server','client'))
    parser.add_argument('--socket',type=Path,required=True)
    parser.add_argument('--output',type=Path,default=Path('tmp/s15_6b/client_result.json'))
    parser.add_argument('--bad-frame',action='store_true');parser.add_argument('--stale',action='store_true')
    parser.add_argument('--bad-pose',action='store_true');parser.add_argument('--nonfinite',action='store_true')
    parser.add_argument('--cancel-after',type=float,default=-1.);parser.add_argument('--cancel-phase',default='',choices=('', 'home_hold','transit','close','lift','transfer','descend','release','final_hold'))
    parser.add_argument('--execution-timeout',type=float,default=120.)
    args,ros_args=parser.parse_known_args()
    if not math.isfinite(args.execution_timeout) or args.execution_timeout<=0:parser.error('positive finite execution timeout')
    if not math.isfinite(args.cancel_after) or (args.cancel_after<0 and args.cancel_after!=-1):parser.error('cancel after -1 or finite nonnegative')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    rclpy.init(args=ros_args);node=None;executor=None
    try:
        if args.role=='perception':node=Perception(args);rclpy.spin(node)
        elif args.role=='server':
            node=Server(args);executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node);executor.spin()
        else:node=sim_node('s15_6b_client');return client(node,args)
        return 0
    except KeyboardInterrupt:return 130
    finally:
        if executor is not None:executor.shutdown()
        if node is not None:
            if isinstance(node,Server):node.action.destroy()
            node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
