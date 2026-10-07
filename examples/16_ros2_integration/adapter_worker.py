"""S15.6a conda-only algorithm worker; JSON over local Unix socket, no ROS imports."""
import argparse
import json
import os
from pathlib import Path
import socket
import sys
import uuid

import mujoco
import mujoco_menagerie
import numpy as np

EXAMPLES=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(EXAMPLES/'14_vision_manipulation'))
sys.path.insert(0,str(EXAMPLES/'12_pick_place'))
from perception_pose import map_estimate, rigid_transform, inverse
from pregrasp_motion import ARM_JOINTS, joint_addresses, solve_pregrasp_ik, cubic_reference, site_pose, orientation_error_world


class Algorithms:
    def __init__(self):
        self.model=mujoco_menagerie.load('universal_robots_ur5e')
        self.data=mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model,self.data,self.model.key('home').id)
        mujoco.mj_forward(self.model,self.data)
        self.aq,self.ad=joint_addresses(self.model,ARM_JOINTS)
        self.act=[self.model.actuator(n).id for n in ('shoulder_pan','shoulder_lift','elbow','wrist_1','wrist_2','wrist_3')]
        self.site=self.model.site('attachment_site').id
        self.plan=None;self.running=False;self.step_index=0
        self.WB=rigid_transform(self.data.body('base').xmat.reshape(3,3),self.data.body('base').xpos)
        self.WC=rigid_transform(np.diag([1.,-1.,-1.]),[-.35,.1,.8])

    def observe(self,stamp):
        # Synthetic producer only; no detector, RGB image or object truth feedback during execution.
        WO=rigid_transform(np.eye(3),[-.45,.2,.03])
        packet=dict(valid=True,pose=(inverse(self.WC)@WO).tolist(),from_frame='object',to_frame='camera_optical',
                    length_unit='m',timestamp_s=stamp,reprojection_rms_px=.1,correspondence_count=12,all_points_positive_depth=True)
        mapped=map_estimate(packet,inverse(self.WB)@self.WC,self.WB,stamp)
        return dict(position=mapped['T_WO'][:3,3].tolist(),quaternion_xyzw=[0.,0.,0.,1.])

    def make_plan(self,position,quaternion):
        if self.running:raise ValueError('execution_busy')
        self.plan=None  # A new planning attempt invalidates the previous idle plan.
        p=np.asarray(position,dtype=float);q=np.asarray(quaternion,dtype=float)
        if p.shape!=(3,) or q.shape!=(4,) or not np.isfinite(p).all() or not np.isfinite(q).all():raise ValueError('invalid_pose')
        if not np.allclose(q,[0,0,0,1],atol=1e-9):raise ValueError('only_upright_identity_object_pose_supported')
        if not (-.6<=p[0]<=-.3 and .1<=p[1]<=.3 and .02<=p[2]<=.06):raise ValueError('object_outside_fixture_workspace')
        target=p+np.array([0.,0.,.22]);rotation=np.diag([1.,-1.,-1.])
        start=self.data.qpos[self.aq].copy()
        private=mujoco.MjData(self.model);private.qpos[:]=self.data.qpos
        goal,updates,ep,er=solve_pregrasp_ik(self.model,private,self.site,self.aq,self.ad,start,target,rotation)
        dt=.02;delta=np.abs(goal-start)
        duration=float(np.ceil(max(float(np.max(1.5*delta/.2)),float(np.max(np.sqrt(6*delta/.8))),1.)/dt)*dt)
        times,positions,velocities=cubic_reference(start,goal,duration,dt)
        if duration>20:raise ValueError('reference_duration_budget')
        self.plan=dict(plan_id=uuid.uuid4().hex,joint_names=list(ARM_JOINTS),times=times.tolist(),positions=positions.tolist(),
                       target=target.tolist(),rotation=rotation.tolist(),ik_updates=updates,ik_position_error_m=ep,ik_rotation_error_rad=er)
        return self.plan

    def begin(self,plan_id):
        if self.running or self.plan is None or self.plan['plan_id']!=plan_id:raise ValueError('no_matching_idle_plan')
        if np.max(np.abs(self.data.qpos[self.aq]-self.plan['positions'][0]))>1e-6:raise ValueError('start_state_changed')
        self.running=True;self.step_index=0
        return dict(started=True)

    def stop(self):
        self.running=False
        self.plan=None  # Consume invalidated plan; cannot resume/replay implicitly.
        return dict(stopped=True,sim_time_s=float(self.data.time),actual=self.data.qpos[self.aq].tolist(),
                    velocity=self.data.qvel[self.ad].tolist())

    def tick(self):
        if not self.running:raise ValueError('no_active_execution')
        k=self.step_index;rows=self.plan['positions'];goal=rows[-1]
        command=np.asarray(rows[min(k,len(rows)-1)])
        self.data.ctrl[self.act]=command
        # Ideal model bias external torque, not a hardware actuator guarantee.
        for _ in range(int(round(.02/self.model.opt.timestep))):
            self.data.qfrc_applied[self.ad]=self.data.qfrc_bias[self.ad]
            mujoco.mj_step(self.model,self.data)
            mujoco.mj_forward(self.model,self.data)
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():raise ValueError('nonfinite_execution')
        tracking=float(np.max(np.abs(command-self.data.qpos[self.aq])))
        if tracking>.06:raise ValueError('tracking_limit')
        self.step_index+=1
        # .6 simulated seconds of final hold. All movement after initialization is ctrl/mj_step.
        done=self.step_index>=len(rows)+30
        p,R=site_pose(self.data,self.site)
        ep=float(np.linalg.norm(p-self.plan['target']));er=float(np.linalg.norm(orientation_error_world(R,np.asarray(self.plan['rotation']))))
        result=dict(done=done,command=command.tolist(),actual=self.data.qpos[self.aq].tolist(),tracking_rad=tracking,
                    progress=min(k/max(1,len(rows)-1),1.),sim_time_s=float(self.data.time),position_error_m=ep,rotation_error_rad=er)
        if done:
            self.running=False;self.plan=None
            if ep>.003 or er>.02:raise ValueError('final_pose_policy')
        return result

    def request(self,message):
        op=message['op']
        if op=='observe':return self.observe(float(message['stamp']))
        if op=='plan':return self.make_plan(message['position'],message['quaternion'])
        if op=='begin':return self.begin(message['plan_id'])
        if op=='tick':return self.tick()
        if op=='stop':return self.stop()
        if op=='state':return dict(sim_time_s=float(self.data.time),actual=self.data.qpos[self.aq].tolist(),running=self.running)
        raise ValueError('unknown_operation')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--socket',type=Path,default=Path('tmp/s15_6a/worker.sock'))
    parser.add_argument('--standalone',action='store_true')
    args=parser.parse_args()
    if os.environ.get('CONDA_DEFAULT_ENV')!='mujoco' or Path(sys.prefix).name!='mujoco':
        raise RuntimeError('worker requires verified conda mujoco; no system Python fallback')
    algorithms=Algorithms()
    if args.standalone:
        observation=algorithms.observe(0.)
        plan=algorithms.make_plan(observation['position'],observation['quaternion_xyzw'])
        algorithms.begin(plan['plan_id']);maximum=0.
        while True:
            result=algorithms.tick();maximum=max(maximum,result['tracking_rad'])
            if result['done']:break
        print(json.dumps(dict(status='standalone_passed',plan_duration_s=plan['times'][-1],max_tracking_rad=maximum,final=result),indent=2))
        return
    args.socket.parent.mkdir(parents=True,exist_ok=True)
    if args.socket.exists():raise RuntimeError('socket already exists; do not replace another worker')
    with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as server:
        server.bind(str(args.socket));server.listen(4)
        print(f'worker_ready: {args.socket}; {sys.executable}',flush=True)
        try:
            while True:
                connection,_=server.accept()
                with connection,connection.makefile('rwb') as stream:
                    try:
                        raw=stream.readline(2_000_000)
                        message=json.loads(raw)
                        response=dict(ok=True,value=algorithms.request(message))
                    except Exception as error:
                        response=dict(ok=False,reason=f'{type(error).__name__}: {error}')
                    stream.write((json.dumps(response,allow_nan=False)+'\n').encode());stream.flush()
        except KeyboardInterrupt:pass
        finally:args.socket.unlink(missing_ok=True)


if __name__=='__main__':main()
