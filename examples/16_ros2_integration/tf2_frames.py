"""S15.4: synthetic frame tree, time lookup and application freshness gate."""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, StaticTransformBroadcaster, TransformListener
from tf2_ros import TransformException


def transform(parent, child, xyz, xyzw, stamp):
    if parent == child or not parent or not child:
        raise ValueError("distinct nonempty frames required")
    if len(xyz) != 3 or len(xyzw) != 4 or not all(math.isfinite(v) for v in (*xyz, *xyzw)):
        raise ValueError("finite xyz(3) and xyzw(4) required")
    if abs(sum(v*v for v in xyzw) - 1) > 1e-9:
        raise ValueError("unit quaternion required")
    msg = TransformStamped()
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.header.stamp = stamp
    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = xyz
    # ROS uses x,y,z,w; MuJoCo freejoint quaternion uses w,x,y,z.
    msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = xyzw
    return msg


def apply(msg, point):
    """p_target = R_target_source p_source + t_target_source; point in meters."""
    q = msg.transform.rotation
    v = (q.x,q.y,q.z)
    def cross(a,b):
        return (a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])
    uv = cross(v,point)
    uuv = cross(v,uv)
    t = msg.transform.translation
    return [point[i]+2*q.w*uv[i]+2*uuv[i]+shift for i,shift in enumerate((t.x,t.y,t.z))]


def close(actual, expected):
    if any(abs(a-b) > 1e-9 for a,b in zip(actual,expected)):
        raise AssertionError((actual,expected))


def fresh_age(now, stamp, max_age):
    age = (now.nanoseconds - Time.from_msg(stamp).nanoseconds)*1e-9
    if age < -0.05:
        raise ValueError(f"future_stamp: age={age:.6f}s")
    if age > max_age:
        raise ValueError(f"stale_transform: age={age:.6f}s > {max_age}s")
    return age


def history_checks():
    # Separate deterministic Buffer: no subscriptions, no changes to live listener state.
    buf = Buffer(cache_time=Duration(seconds=5))
    for second,x in ((10,0.0),(12,2.0)):
        msg = transform('history_world','history_object',(x,0.,0.),(0.,0.,0.,1.),Time(seconds=second).to_msg())
        buf.set_transform(msg,'s15_4_history_fixture')  # In-place cache insertion.
    midpoint = buf.lookup_transform('history_world','history_object',Time(seconds=11))
    close(apply(midpoint,[0.,0.,0.]),[1.,0.,0.])
    close(apply(buf.lookup_transform('history_object','history_world',Time(seconds=11)),[1.,0.,0.]),[0.,0.,0.])
    for second in (9,13):
        try:
            buf.lookup_transform('history_world','history_object',Time(seconds=second))
        except TransformException:
            pass
        else:
            raise AssertionError('out-of-range time should fail')
    print('history_passed=True; midpoint_x=1m; inverse/past/future_checks_passed')


class FramePublisher(Node):
    def __init__(self,args):
        super().__init__('s15_4_broadcaster')
        self.args = args
        self.started = time.monotonic()
        self.static = StaticTransformBroadcaster(self)
        self.dynamic = TransformBroadcaster(self)
        stamp = self.get_clock().now().to_msg()
        # T_WB from S12.1 historical UR5e convention, not a fresh MuJoCo measurement.
        wb = transform('world','base',(0.,0.,0.),(0.,0.,1.,0.),stamp)
        bc = transform('base','camera_optical',(-args.camera_x,-.1,.8),(0.,1.,0.,0.),stamp)
        # Send both together; /tf_static transient-local supplies these to late listeners.
        self.static.sendTransform([wb,bc])
        self.timer = self.create_timer(.05,self.publish)

    def publish(self):
        elapsed = time.monotonic()-self.started
        if self.args.stop_after >= 0 and elapsed >= self.args.stop_after:
            return  # Node stays alive; latest cached dynamic transform can become stale.
        stamp = (self.get_clock().now()-Duration(seconds=self.args.stamp_lag)).to_msg()
        # Fixed object world origin [-.45,.2,.03]m, optical axes +X,-Y,-Z in world.
        co = transform('camera_optical','object',(-.45-self.args.camera_x,-.1,.77),(1.,0.,0.,0.),stamp)
        bt = transform('base','tool',(.3+.02*math.sin(elapsed),0.,.4),(0.,0.,0.,1.),stamp)
        self.dynamic.sendTransform([co,bt])


def probe(node,args):
    buf = Buffer(cache_time=Duration(seconds=5),node=node)
    listener = TransformListener(buf,node)  # Subscribes to /tf and /tf_static; requires spin.
    deadline = time.monotonic()+args.timeout
    latest = None
    while rclpy.ok() and time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.05)
        try:
            latest = buf.lookup_transform('world','object',Time())  # target first; zero means latest common time.
            tool = buf.lookup_transform('base','tool',Time())
            break
        except TransformException:
            continue
    if latest is None or time.monotonic() >= deadline:
        print('status=transform_unavailable')
        return 3
    observe_until = time.monotonic()+args.observe_for
    while time.monotonic()<observe_until:
        rclpy.spin_once(node,timeout_sec=.05)
    latest = buf.lookup_transform('world','object',Time())
    tool = buf.lookup_transform('base','tool',Time())
    print(f'latest_lookup_succeeded=True; stamp={latest.header.stamp.sec}.{latest.header.stamp.nanosec:09d}')
    try:
        now = node.get_clock().now()
        object_age = fresh_age(now,latest.header.stamp,args.max_age)
        tool_age = fresh_age(now,tool.header.stamp,args.max_age)
    except ValueError as error:
        print(f'status=transform_rejected; {error}')
        return 1
    stamp = Time.from_msg(latest.header.stamp)
    bo = buf.lookup_transform('base','object',stamp)
    ob = buf.lookup_transform('object','base',stamp)
    # Non-origin point catches rotation mistakes that a pure translation test misses.
    point_o = [.02,.01,.03]
    point_b = apply(bo,point_o)
    point_w = apply(latest,point_o)
    close(point_b,[.43,-.21,.06]);close(point_w,[-.43,.21,.06])
    close(apply(ob,point_b),point_o)
    print(f'world_object_origin={apply(latest,[0.,0.,0.])}; base_object_point={point_b}')
    print(f'object_age_s={object_age:.6f}; tool_age_s={tool_age:.6f}; freshness_passed=True')
    # A static edge is valid at arbitrary times; do not age-gate its stamp as a dynamic measurement.
    static = buf.lookup_transform('base','camera_optical',Time(seconds=1))
    close(apply(static,[0.,0.,0.]),[-args.camera_x,-.1,.8])
    try:
        buf.lookup_transform('world','unknown_frame',Time())
    except TransformException:
        print('unknown_frame_rejected=True')
    else:
        raise AssertionError('unknown frame unexpectedly accepted')
    history_checks()
    print('status=frame_time_checks_passed')
    return 0


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('role',choices=('broadcaster','probe'))
    parser.add_argument('--camera-x',type=float,default=-.35)
    parser.add_argument('--stamp-lag',type=float,default=0.)
    parser.add_argument('--stop-after',type=float,default=-1.)
    parser.add_argument('--max-age',type=float,default=.3)
    parser.add_argument('--timeout',type=float,default=5.)
    parser.add_argument('--observe-for',type=float,default=0.,help='keep the listener alive before age check [s]')
    args,ros_args=parser.parse_known_args()
    if not all(math.isfinite(v) for v in (args.camera_x,args.stamp_lag,args.stop_after,args.max_age,args.timeout,args.observe_for)):
        parser.error('all values must be finite')
    if not 0 <= args.stamp_lag <= 2 or args.max_age <= 0 or args.timeout <= 0 or args.observe_for<0 or (args.stop_after<0 and args.stop_after!=-1):
        parser.error('lag 0..2s; max-age/timeout positive; stop-after -1 or nonnegative')
    rclpy.init(args=ros_args)
    node=None
    try:
        node=FramePublisher(args) if args.role=='broadcaster' else Node('s15_4_probe')
        if args.role=='probe':return probe(node,args)
        print('tree_ready: world->base->camera_optical->object; base->tool',flush=True)
        rclpy.spin(node)
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':
    raise SystemExit(main())
