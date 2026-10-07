"""S15.6a system-Python ROS adapters; no MuJoCo/NumPy imports."""
import argparse
import json
import math
from pathlib import Path
import socket
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient,ActionServer,GoalResponse,CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from s15_interfaces.srv import PlanPose

TOPIC='/s15_6a/object_pose'
SERVICE='/s15_6a/plan_pose'
ACTION='/s15_6a/execute'


def rpc(path,message):
    with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as connection:
        connection.settimeout(3.)
        connection.connect(str(path))
        with connection.makefile('rwb') as stream:
            stream.write((json.dumps(message,allow_nan=False)+'\n').encode());stream.flush()
            response=json.loads(stream.readline(2_000_000))
    if not response['ok']:raise ValueError(response['reason'])
    return response['value']


def stamp_seconds(stamp):return stamp.sec+stamp.nanosec*1e-9


def pose_fields(pose):
    p=pose.pose.position;q=pose.pose.orientation
    return [p.x,p.y,p.z],[q.x,q.y,q.z,q.w]


def trajectory(plan):
    result=JointTrajectory();result.joint_names=plan['joint_names']
    for t,q in zip(plan['times'],plan['positions']):
        point=JointTrajectoryPoint();point.positions=q
        ns=round(t*1e9);point.time_from_start.sec=ns//1000000000;point.time_from_start.nanosec=ns%1000000000
        result.points.append(point)
    return result


class Perception(Node):
    def __init__(self,args):
        super().__init__('s15_6a_perception');self.args=args
        self.publisher=self.create_publisher(PoseStamped,TOPIC,10)
        self.timer=self.create_timer(.1,self.publish)

    def publish(self):
        stamp=self.get_clock().now().to_msg()
        observation=rpc(self.args.socket,dict(op='observe',stamp=stamp_seconds(stamp)))
        msg=PoseStamped();msg.header.frame_id='world';msg.header.stamp=stamp
        p=observation['position'];q=observation['quaternion_xyzw']
        msg.pose.position.x,msg.pose.position.y,msg.pose.position.z=p
        msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w=q
        self.publisher.publish(msg)


class Adapters(Node):
    def __init__(self,args):
        super().__init__('s15_6a_adapters');self.args=args
        self.lock=threading.Lock();self.busy=False;self.plan=None;self.expected=None
        group=ReentrantCallbackGroup()
        self.service=self.create_service(PlanPose,SERVICE,self.plan_pose,callback_group=group)
        self.action=ActionServer(self,FollowJointTrajectory,ACTION,execute_callback=self.execute,
                                goal_callback=self.accept,cancel_callback=self.cancel,callback_group=group)

    def plan_pose(self,request,response):
        # Short bounded IK/cubic callback. Never run the full action inside this service.
        with self.lock:
            if self.busy:
                response.reason='execution_busy';return response
            self.plan=None;self.expected=None
            try:
                msg=request.object_pose
                age=stamp_seconds(self.get_clock().now().to_msg())-stamp_seconds(msg.header.stamp)
                if msg.header.frame_id!='world':raise ValueError('expected_world_frame')
                if not 0<=age<=.5:raise ValueError('future_or_stale_observation')
                p,q=pose_fields(msg)
                if not all(math.isfinite(v) for v in (*p,*q)):raise ValueError('nonfinite_pose')
                start=time.monotonic()
                plan=rpc(self.args.socket,dict(op='plan',position=p,quaternion=q))
                elapsed=time.monotonic()-start
                if elapsed>2:raise ValueError('short_planning_budget_exceeded')
                self.plan=plan;self.expected=trajectory(plan)
                response.success=True;response.reason=f'ik_cubic_reference_ready; planning_wall_s={elapsed:.6f}'
                response.plan_id=plan['plan_id'];response.trajectory=self.expected
                self.get_logger().info(f'plan_ready: {plan["plan_id"]}; duration_s={plan["times"][-1]}')
            except Exception as error:
                response.success=False;response.reason=str(error)
                self.get_logger().info(f'plan_rejected: {error}')
        return response

    def accept(self,goal):
        # Accept only the exact issued plan and supported standard action fields.
        with self.lock:
            if (self.busy or self.plan is None or goal.trajectory!=self.expected
                    or goal.multi_dof_trajectory.points or goal.multi_dof_trajectory.joint_names
                    or goal.path_tolerance or goal.goal_tolerance or goal.component_path_tolerance
                    or goal.component_goal_tolerance or goal.goal_time_tolerance.sec or goal.goal_time_tolerance.nanosec):
                self.get_logger().info('goal_rejected: busy/no matching issued reference/unsupported fields')
                return GoalResponse.REJECT
            self.busy=True
        return GoalResponse.ACCEPT

    def cancel(self,handle):return CancelResponse.ACCEPT

    def execute(self,handle):
        result=FollowJointTrajectory.Result();maximum=0.
        try:
            rpc(self.args.socket,dict(op='begin',plan_id=self.plan['plan_id']))
            while True:
                if handle.is_cancel_requested:
                    stopped=rpc(self.args.socket,dict(op='stop'))
                    handle.canceled();result.error_string=json.dumps(dict(status='simulation_paused',state=stopped))
                    self.get_logger().info(f'terminal=CANCELED; sim_time={stopped["sim_time_s"]}')
                    return result
                state=rpc(self.args.socket,dict(op='tick'))
                maximum=max(maximum,state['tracking_rad'])
                feedback=FollowJointTrajectory.Feedback();feedback.joint_names=self.expected.joint_names
                feedback.header.stamp=self.get_clock().now().to_msg()
                feedback.desired.positions=state['command'];feedback.actual.positions=state['actual']
                feedback.error.positions=[a-b for a,b in zip(state['command'],state['actual'])]
                handle.publish_feedback(feedback)
                if state['done']:
                    handle.succeed();result.error_code=result.SUCCESSFUL
                    result.error_string=json.dumps(dict(status='execution_passed',max_tracking_rad=maximum,final=state))
                    self.get_logger().info('terminal=SUCCEEDED')
                    return result
                time.sleep(.005)  # wall pacing for observable feedback/cancel, not realtime scheduling
        except Exception as error:
            # Request stop before returning ABORTED; failed IPC leaves remote state uncertain.
            try:rpc(self.args.socket,dict(op='stop'));stop_state='simulation_paused'
            except Exception:stop_state='remote_state_unknown'
            if handle.is_active:handle.abort()
            result.error_code=result.PATH_TOLERANCE_VIOLATED;result.error_string=f'{error}; {stop_state}'
            return result
        finally:
            with self.lock:self.busy=False;self.plan=None;self.expected=None


def wait(node,future,seconds):
    rclpy.spin_until_future_complete(node,future,timeout_sec=seconds)
    if not future.done():raise TimeoutError('ROS future timeout; remote state unknown')
    return future.result()


def run_client(node,args):
    received=[]
    subscription=node.create_subscription(PoseStamped,TOPIC,lambda msg:received.append(msg),10)
    deadline=time.monotonic()+5
    while not received and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
    if not received:print('status=perception_unavailable');return 3
    pose=received[-1]
    if args.bad_frame:pose.header.frame_id='camera_optical'
    if args.stale:pose.header.stamp.sec-=2
    client=node.create_client(PlanPose,SERVICE)
    if not client.wait_for_service(timeout_sec=3):print('status=planning_service_unavailable');return 3
    request=PlanPose.Request();request.object_pose=pose
    response=wait(node,client.call_async(request),4)
    print(f'planning_success={response.success}; reason={response.reason}',flush=True)
    if not response.success:return 1  # No execution request on failure.
    print(f'plan_id={response.plan_id}; points={len(response.trajectory.points)}',flush=True)
    action=ActionClient(node,FollowJointTrajectory,ACTION)
    if not action.wait_for_server(timeout_sec=3):return 3
    goal=FollowJointTrajectory.Goal();goal.trajectory=response.trajectory
    count=0
    def feedback(message):
        nonlocal count
        count+=1
        if count%25==0:print(f'feedback_count={count}; actual_q0={message.feedback.actual.positions[0]:.6f}',flush=True)
    handle=wait(node,action.send_goal_async(goal,feedback_callback=feedback),3)
    if not handle.accepted:print('status=execution_rejected');return 2
    print('execution_goal_accepted=True',flush=True)
    future=handle.get_result_async();start=time.monotonic();cancel=None
    while not future.done() and time.monotonic()-start<30:
        rclpy.spin_once(node,timeout_sec=.02)
        if args.cancel_after>=0 and cancel is None and time.monotonic()-start>=args.cancel_after:
            cancel=handle.cancel_goal_async()
    if not future.done():
        handle.cancel_goal_async();print('status=result_timeout; cancel_requested; remote_state_unknown');return 4
    if cancel is not None:
        acknowledgement=wait(node,cancel,2)
        print(f'cancel_acknowledged={bool(acknowledgement.goals_canceling)}')
    wrapped=future.result()
    name={4:'SUCCEEDED',5:'CANCELED',6:'ABORTED'}.get(wrapped.status,str(wrapped.status))
    print(f'terminal={name}; code={wrapped.result.error_code}; feedback_count={count}; result={wrapped.result.error_string}')
    if args.cancel_after>=0:return 0 if wrapped.status==GoalStatus.STATUS_CANCELED else 1
    return 0 if wrapped.status==GoalStatus.STATUS_SUCCEEDED and wrapped.result.error_code==0 else 1


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('role',choices=('perception','server','client'))
    parser.add_argument('--socket',type=Path,default=Path('tmp/s15_6a/worker.sock'))
    parser.add_argument('--bad-frame',action='store_true');parser.add_argument('--stale',action='store_true')
    parser.add_argument('--cancel-after',type=float,default=-1.)
    args,ros_args=parser.parse_known_args()
    if not math.isfinite(args.cancel_after) or (args.cancel_after<0 and args.cancel_after!=-1):parser.error('cancel-after -1 or finite nonnegative seconds')
    rclpy.init(args=ros_args);node=None;executor=None
    try:
        if args.role=='perception':node=Perception(args);rclpy.spin(node)
        elif args.role=='server':
            node=Adapters(args);executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node);executor.spin()
        else:node=Node('s15_6a_client');return run_client(node,args)
        return 0
    except KeyboardInterrupt:return 130
    finally:
        if executor is not None:executor.shutdown()
        if node is not None:
            if isinstance(node,Adapters):node.action.destroy()
            node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
