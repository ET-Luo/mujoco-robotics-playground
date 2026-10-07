"""Conda-only S15.6b image/PnP + existing continuous manipulation pipeline."""
import argparse
import json
import os
from pathlib import Path
import socket
import sys
import threading
import uuid

import cv2
import mujoco
import numpy as np

EXAMPLES=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(EXAMPLES/'14_vision_manipulation'))
sys.path.insert(0,str(EXAMPLES/'12_pick_place'))
from vision_pick_place import (run_pipeline,save_artifacts,build_model,PHASES,ARM_JOINTS,
                              joint_addresses,rigid_transform,inverse,map_estimate,plan_motion,DESTINATION)
from vision_to_motion import (make_image,detect_pixels,estimate_pose,POINTS,K,DISTORTION,
                             project,rms_2d,upright_estimate)


class Cancelled(RuntimeError):
    pass


def serial(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    raise TypeError(type(value).__name__)


class Manipulation:
    def __init__(self,out,close_target):
        cv2.setNumThreads(1)
        self.out=out;out.mkdir(parents=True,exist_ok=True)
        self.close_target=close_target
        self.plan_model=build_model();self.model=build_model(free_object=True)
        base=mujoco.MjData(self.plan_model);mujoco.mj_forward(self.plan_model,base)
        WB=rigid_transform(base.body('base').xmat.reshape(3,3),base.body('base').xpos)
        WC=rigid_transform(np.diag([1.,-1.,-1.]),[-.35,.1,.8])
        truth=rigid_transform(np.eye(3),[-.45,.2,.03])  # Only producer; no held-object target feedback.
        image=make_image(inverse(WC)@truth,.2)
        if not cv2.imwrite(str(out/'landmarks.png'),image):raise RuntimeError('landmark_image_save_failed')
        pixels=detect_pixels(image)
        CO,rvec,tvec=estimate_pose(POINTS,pixels,K,DISTORTION)
        rms=rms_2d(project(POINTS,rvec,tvec,K,DISTORTION)-pixels)
        packet=dict(valid=True,pose=CO,from_frame='object',to_frame='camera_optical',length_unit='m',timestamp_s=0.,
                    reprojection_rms_px=rms,correspondence_count=len(pixels),all_points_positive_depth=True)
        raw=map_estimate(packet,inverse(WB)@WC,WB,0.)['T_WO']
        self.estimate,tilt=upright_estimate(raw)
        self.perception=dict(rms_px=float(rms),upright_tilt_deg=float(tilt),producer='colored_landmark_image_PnP')
        quat=np.zeros(4);mujoco.mju_mat2Quat(quat,self.estimate[:3,:3].ravel())
        self.observation=dict(position=self.estimate[:3,3].tolist(),quaternion_xyzw=quat[[1,2,3,0]].tolist())
        self.aq,self.ad=joint_addresses(self.model,ARM_JOINTS)
        self.condition=threading.Condition();self.budget=0;self.steps=0
        self.cancel=False;self.running=False;self.done=False;self.used=False;self.plan=None
        self.result={};self.trace=[];self.arrays={};self.data=None;self.thread=None
        self.state=dict(phase='idle',simulation_time_s=0.,actual_joint_positions=self.model.key('home').qpos[self.aq].tolist(),
                        actual_joint_velocities=[0.]*6,tool_position_world_m=None,tool_rotation_world=None)

    def prepare(self,position,xyzw):
        if self.running or self.used:raise ValueError('single_trial_worker_requires_restart')
        self.plan=None
        p=np.asarray(position,dtype=float);q=np.asarray(xyzw,dtype=float)
        if p.shape!=(3,) or q.shape!=(4,) or not np.isfinite(p).all() or not np.isfinite(q).all():raise ValueError('invalid_pose')
        if abs(np.linalg.norm(q)-1)>1e-6:raise ValueError('nonunit_orientation')
        if np.linalg.norm(p-[-.45,.2,.03])>.015:raise ValueError('outside_fixed_scene_pose_gate')
        rotation=np.zeros(9);mujoco.mju_quat2Mat(rotation,q[[3,0,1,2]])
        estimate,_=upright_estimate(rigid_transform(rotation.reshape(3,3),p))
        candidate,home,reference,peak=plan_motion(self.plan_model,estimate)
        self.plan=dict(plan_id=uuid.uuid4().hex,candidate=candidate,home=home,reference=reference,peak=peak,estimate=estimate)
        return dict(plan_id=self.plan['plan_id'],candidate_id=candidate['id'],reference_samples=len(reference),
                    approach_reference_peak_rad_s=peak)

    def guard(self,phase,data):
        # Called before every control write/mj_step in reused run_pipeline.
        with self.condition:
            self.data=data
            self.condition.wait_for(lambda:self.cancel or self.budget>0)
            if self.cancel:raise Cancelled('client_cancel_requested_before_next_physics_step')
            self.budget-=1

    def observe(self,phase,data,allowed):
        with self.condition:
            self.steps+=1
            self.state=dict(phase=phase,simulation_time_s=float(data.time),
                            actual_joint_positions=data.qpos[self.aq].tolist(),actual_joint_velocities=data.qvel[self.ad].tolist(),
                            tool_position_world_m=data.site('attachment_site').xpos.tolist(),
                            tool_rotation_world=data.site('attachment_site').xmat.reshape(3,3).tolist(),
                            all_qpos=data.qpos.tolist(),all_qvel=data.qvel.tolist())
            self.condition.notify_all()

    def execute(self):
        plan=self.plan
        self.result=dict(status='failed',phase='planning',simulation_time_s=0.,phases=list(PHASES),stages={},
                         approach_peak_reference_speed_rad_s=plan['peak'],candidate_id=plan['candidate']['id'],
                         perception=self.perception,plan_id=plan['plan_id'],close_target_m=self.close_target,
                         destination_W_m=DESTINATION.tolist())
        self.arrays=dict(approach_reference=plan['reference'],T_WO_estimate=plan['estimate'])
        try:
            run_pipeline(self.plan_model,self.model,plan['candidate'],plan['home'],plan['reference'],self.close_target,
                         self.trace,self.result,geometry_observer=self.observe,execution_guard=self.guard)
        except Cancelled as error:
            self.result.update(status='cancelled',failure_reason=str(error))
        except Exception as error:
            self.result.update(status='failed',failure_reason=f'{type(error).__name__}: {error}')
        finally:
            # Save partial evidence for cancel/failure as well as successful placement.
            try:save_artifacts(self.out,self.result,self.trace,self.arrays)
            except Exception as error:self.result.update(status='failed',artifact_error=str(error))
            with self.condition:
                self.running=False;self.done=True;self.plan=None
                self.condition.notify_all()

    def begin(self,plan_id):
        if self.running or self.used or self.plan is None or self.plan['plan_id']!=plan_id:raise ValueError('invalid_or_consumed_plan')
        self.running=True;self.used=True
        self.thread=threading.Thread(target=self.execute,daemon=True);self.thread.start()
        return dict(started=True)

    def tick(self):
        with self.condition:
            if not self.running and not self.done:raise ValueError('no_active_trial')
            goal=self.steps+10
            if self.running:
                self.budget+=10;self.condition.notify_all()
                if not self.condition.wait_for(lambda:self.done or self.steps>=goal,timeout=15):raise TimeoutError('worker_step_budget_timeout')
            return self.snapshot()

    def stop(self):
        with self.condition:self.cancel=True;self.condition.notify_all()
        if self.thread is not None:
            self.thread.join(timeout=15)
            if self.thread.is_alive():raise TimeoutError('remote_execution_not_confirmed_stopped')
        return self.snapshot()

    def snapshot(self):
        return dict(state=self.state.copy(),steps=self.steps,running=self.running,done=self.done,
                    result=self.result if self.done else None)

    def request(self,message):
        op=message['op']
        if op=='observe':return self.observation
        if op=='plan':return self.prepare(message['position'],message['quaternion'])
        if op=='begin':return self.begin(message['plan_id'])
        if op=='tick':return self.tick()
        if op=='stop':return self.stop()
        if op=='state':
            with self.condition:return self.snapshot()
        raise ValueError('unknown_operation')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--socket',type=Path,required=True);parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--close-target',type=float,default=.005)
    args=parser.parse_args()
    if os.environ.get('CONDA_DEFAULT_ENV')!='mujoco' or Path(sys.prefix).name!='mujoco':raise RuntimeError('conda mujoco required')
    if not np.isfinite(args.close_target) or not 0<=args.close_target<=.04:parser.error('close target finite [0,.04]m')
    worker=Manipulation(args.out,args.close_target)
    if args.socket.exists():raise RuntimeError('refuse replacing existing worker socket')
    args.socket.parent.mkdir(parents=True,exist_ok=True)
    with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as server:
        server.bind(str(args.socket));server.listen(4)
        print(f'worker_ready: {args.socket}; {sys.executable}',flush=True)
        try:
            while True:
                connection,_=server.accept()
                with connection,connection.makefile('rwb') as stream:
                    try:response=dict(ok=True,value=worker.request(json.loads(stream.readline(2_000_000))))
                    except Exception as error:response=dict(ok=False,reason=f'{type(error).__name__}: {error}')
                    stream.write((json.dumps(response,default=serial,allow_nan=False)+'\n').encode());stream.flush()
        except KeyboardInterrupt:worker.stop()
        finally:args.socket.unlink(missing_ok=True)


if __name__=='__main__':main()
