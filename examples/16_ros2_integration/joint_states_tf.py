"""ROS2 system Python ONLY: named JointState -> robot_state_publisher -> TF/FK check."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_prefix
from tf2_ros import Buffer,TransformListener,TransformException

URDF=Path(__file__).with_name('ur5e_kinematics.urdf')


def matrix(msg):
    t=msg.transform.translation;q=msg.transform.rotation
    x,y,z,w=q.x,q.y,q.z,q.w
    return [[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),t.x],
            [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),t.y],
            [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y),t.z],
            [0.,0.,0.,1.]]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference',type=Path,default=Path('tmp/s15_5_urdf/reference.json'))
    parser.add_argument('--reverse-order',action='store_true')
    parser.add_argument('--wrong-order',action='store_true',help='fault: reverse names without corresponding values')
    args,ros_args=parser.parse_known_args()
    payload=json.loads(args.reference.read_text())
    names=payload['joint_names']
    if len(names)!=6 or len(set(names))!=6:raise ValueError('six unique joint names required')
    for case in payload['cases']:
        if len(case['position'])!=6 or not all(math.isfinite(v) for v in case['position']):raise ValueError('six finite positions required')
    # The external C++ node does URDF FK; the Python node publishes state and observes TF.
    executable=Path(get_package_prefix('robot_state_publisher'))/'lib/robot_state_publisher/robot_state_publisher'
    server=subprocess.Popen([str(executable),
        '--ros-args','-p','robot_description:='+URDF.read_text()],stdout=subprocess.DEVNULL)
    rclpy.init(args=ros_args);node=Node('s15_5_joint_state_probe')
    buf=Buffer(node=node);listener=TransformListener(buf,node)
    publisher=node.create_publisher(JointState,'/joint_states',10)
    maximum=0.
    try:
        for case in payload['cases']:
            # Validate name/position association; arrays can be reordered only together.
            msg=JointState();msg.name=list(names);msg.position=list(case['position'])
            if args.reverse_order or args.wrong_order:msg.name.reverse()
            if args.reverse_order and not args.wrong_order:msg.position.reverse()
            # velocity/effort left empty: unknown, not measured zeros.
            deadline=time.monotonic()+6;passed=False
            while time.monotonic()<deadline:
                if server.poll() is not None:raise RuntimeError('robot_state_publisher exited')
                msg.header.stamp=node.get_clock().now().to_msg()
                publisher.publish(msg)
                # Let RSP compute and broadcast this timestamp before exact-time queries.
                wait=time.monotonic()+.12
                while time.monotonic()<wait:rclpy.spin_once(node,timeout_sec=.02)
                errors=[]
                try:
                    for frame,expected in case['poses'].items():
                        actual=matrix(buf.lookup_transform('world',frame,Time.from_msg(msg.header.stamp)))
                        errors.append(max(abs(actual[i][j]-expected[i][j]) for i in range(4) for j in range(4)))
                except TransformException:
                    continue
                error=max(errors);maximum=max(maximum,error)
                if error>1e-8:
                    print(f'case={case["label"]}; fk_rejected=True; max_matrix_error={error:.6e}')
                    return 1
                print(f'case={case["label"]}; all_frames_passed=True; max_matrix_error={error:.3e}')
                passed=True;break
            if not passed:
                print(f'case={case["label"]}; transform_timeout=True')
                return 3
        print(f'PASS JointState -> URDF FK -> TF matches MuJoCo snapshots; max_error={maximum:.3e}')
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        server.terminate()
        try:server.wait(timeout=3)
        except subprocess.TimeoutExpired:server.kill();server.wait()


if __name__=='__main__':raise SystemExit(main())
