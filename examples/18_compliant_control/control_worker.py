"""S17.9 conda-only worker: one physics owner, bounded mailboxes, no ROS or tick RPC."""
import argparse
import json
import math
import os
from pathlib import Path
import queue
import socket
import sys
import threading
import time
import uuid

import mujoco
import numpy as np
from ur5e_surface_following import (build_model, initialize, measure, orientation_error_world,
    DT, R_WS, ORIGIN, ROTATION_GOAL, K_TRANSLATION, D_TRANSLATION, K_ROTATION, D_ROTATION,
    START_NORMAL, APPROACH_SPEED, FORCE_GAIN_SCALE, JOINT_SPEED_BUDGET, SITE_SPEED_BUDGET,
    FORCE_BUDGET, TARGET, REF_MIN, REF_MAX, SPEED_CAP, force_velocity, scan_reference, JOINTS)


def goal_valid(distance, duration, timeout):
    return all(math.isfinite(v) for v in (distance, duration, timeout)) and .01 <= distance <= .03 and 2 <= duration <= 3 and .25 <= timeout <= 1


class Controller:
    """Only the worker main thread may read/write MjData or call these methods."""
    def __init__(self):
        self.model, self.qa, self.va, self.aid, self.gear, self.caps = build_model()
        self.data, self.initialization = initialize(self.model, self.qa, self.va)
        self.site = self.model.site('scan_tip').id
        self.jp, self.jr, self.mass = np.zeros((3, 6)), np.zeros((3, 6)), np.empty((6, 6))
        self.jid = [self.model.joint(n).id for n in JOINTS]
        self.phase, self.status, self.reason = 4, 'idle', ''
        self.generation, self.used, self.paused = '', False, False
        self.start_sim, self.deadline, self.issued, self.sequence = 0., 0., None, -1
        self.accepted, self.rejected, self.last_rejection = 0, 0, ''
        self.normal_ref, self.distance, self.duration, self.timeout = START_NORMAL, .03, 2., .5
        self.reference = R_WS.T @ (self.data.site_xpos[self.site]-ORIGIN)
        self.hold_x, self.hold_rotation = self.reference.copy(), self.data.site_xmat[self.site].reshape(3, 3).copy()
        self.force = 0.
        self.events, self.rows = [], []
        self.metrics = dict(scan_tangent_error_m=0., scan_force_error_N=0., scan_binormal_error_m=0.,
                            scan_orientation_error_rad=0., scan_contact_all=True, saturated_samples=0,
                            max_dynamics_residual_Nm=0., tail_tangent_error_m=0., tail_force_error_N=0.)

    def event(self, reason):
        self.events.append(dict(event=reason, sim_time_s=float(self.data.time),
                                task_elapsed_s=float(self.data.time-self.start_sim), wall_monotonic_s=time.monotonic()))

    def local_hold(self, status, reason):
        # Latch ACTUAL pose, clear task intent; continue dynamics, including residual motion.
        self.hold_x = R_WS.T @ (self.data.site_xpos[self.site]-ORIGIN)
        self.hold_rotation = self.data.site_xmat[self.site].reshape(3, 3).copy()
        self.reference = self.hold_x.copy()
        self.phase, self.status, self.reason = 4, status, reason
        self.event(reason)

    def lifecycle(self, packet):
        op = packet['op']
        if op == 'begin':
            distance, duration, timeout = (float(packet[k]) for k in ('distance_m', 'leg_duration_s', 'command_timeout_s'))
            if self.used or self.paused or not goal_valid(distance, duration, timeout):
                raise ValueError('one_goal_per_worker; require finite bounded goal and unpaused state')
            self.used = True
            self.distance, self.duration, self.timeout = distance, duration, timeout
            self.start_sim, self.deadline = float(self.data.time), time.monotonic()+timeout
            self.generation, self.phase, self.status, self.reason = uuid.uuid4().hex, 0, 'active', ''
            self.normal_ref = START_NORMAL
            self.event('begin')
        elif op == 'cancel':
            if packet.get('generation') != self.generation or self.status != 'active':
                raise ValueError('no_matching_active_goal')
            self.local_hold('canceled', 'cancel_applied')
        elif op == 'pause':
            if not isinstance(packet.get('paused'), bool):
                raise ValueError('paused must be bool')
            if self.status == 'active':
                raise ValueError('finish_or_cancel_task_before_pause')
            self.paused = packet['paused']
            self.event('pause' if self.paused else 'resume')
        else:
            raise ValueError('unknown_lifecycle_operation; no tick API')
        return self.snapshot()

    def reference_packet(self, packet):
        try:
            if self.status != 'active' or packet.get('generation') != self.generation:
                raise ValueError('inactive_or_wrong_generation')
            seq, issued = packet['sequence'], float(packet['issued_monotonic_s'])
            if isinstance(seq, bool) or not isinstance(seq, int) or seq <= self.sequence:
                raise ValueError('old_or_invalid_sequence')
            age = time.monotonic()-issued
            if not math.isfinite(issued) or not 0 <= age <= self.timeout:
                raise ValueError('expired_or_future_reference')
            values = [float(packet[k]) for k in ('distance_m', 'leg_duration_s', 'normal_target_N')]
            if not all(math.isfinite(v) for v in values) or values != [self.distance, self.duration, TARGET]:
                raise ValueError('reference_descriptor_mismatch')
            self.sequence, self.issued, self.deadline = seq, issued, issued+self.timeout
            self.accepted += 1
        except (ValueError, KeyError, TypeError) as error:
            self.rejected += 1
            self.last_rejection = str(error)

    def snapshot(self):
        return dict(generation=self.generation, status=self.status, reason=self.reason, phase=self.phase, paused=self.paused,
                    sim_time_s=float(self.data.time), task_elapsed_s=float(self.data.time-self.start_sim) if self.used else 0.,
                    snapshot_monotonic_s=time.monotonic(), command_age_s=time.monotonic()-self.issued if self.issued is not None else -1.,
                    accepted_references=self.accepted, rejected_references=self.rejected, last_reference_rejection=self.last_rejection,
                    q_rad=self.data.qpos[self.qa].tolist(), qvel_rad_s=self.data.qvel[self.va].tolist(),
                    actual_surface_m=(R_WS.T @ (self.data.site_xpos[self.site]-ORIGIN)).tolist(),
                    reference_surface_m=self.reference.tolist(), normal_force_N=self.force,
                    metrics=self.metrics.copy())

    def step(self):
        m, d = self.model, self.data
        mujoco.mj_forward(m, d)  # Previous ctrl input measurement, current actual state.
        sensed, active, _ = measure(m, d)
        fn = float((R_WS.T @ sensed)[1])
        gate = bool(active and fn > .05)
        mujoco.mj_jacSite(m, d, self.jp, self.jr, self.site)  # World (3,nv) linear/angular J.
        x, v, omega = R_WS.T @ (d.site_xpos[self.site]-ORIGIN), R_WS.T @ (self.jp @ d.qvel), self.jr @ d.qvel
        elapsed = float(d.time-self.start_sim)
        if self.status == 'active' and time.monotonic() > self.deadline:
            self.local_hold('failed', 'command_timeout')
        if self.status in ('active', 'succeeded'):
            if fn > FORCE_BUDGET:
                self.local_hold('failed', 'force_budget_exceeded')
            elif np.max(np.abs(d.qvel[self.va])) > JOINT_SPEED_BUDGET or np.linalg.norm(v) > SITE_SPEED_BUDGET:
                self.local_hold('failed', 'measured_speed_exceeded')
            elif np.any(d.qpos[self.qa] < m.jnt_range[self.jid, 0]) or np.any(d.qpos[self.qa] > m.jnt_range[self.jid, 1]):
                self.local_hold('failed', 'joint_range_exceeded')
        if self.phase == 0 and gate:
            self.phase = 1; self.event('touch')
        if self.phase in (1, 2, 3) and not gate:
            self.local_hold('failed', 'contact_lost')
        if self.phase == 0 and elapsed >= 3:
            self.local_hold('failed', 'search_timeout')
        if self.phase == 1 and elapsed >= 5-1e-9:
            if abs(fn-TARGET) > .05:
                self.local_hold('failed', 'load_not_ready')
            else:
                self.phase = 2; self.event('scan_start')
        if self.phase == 2 and elapsed >= 5+2*self.duration-1e-9:
            self.phase = 3; self.event('scan_reference_complete')
        vn = 0.
        if self.phase == 0:
            self.normal_ref = max(.015, START_NORMAL-APPROACH_SPEED*elapsed)
            vn = -APPROACH_SPEED if START_NORMAL-APPROACH_SPEED*elapsed > .015 else 0.
        elif self.phase in (1, 2, 3):
            old = self.normal_ref
            self.normal_ref = float(np.clip(old+DT*np.clip(FORCE_GAIN_SCALE*force_velocity(TARGET, fn), -SPEED_CAP, SPEED_CAP), REF_MIN, REF_MAX))
            vn = (self.normal_ref-old)/DT
        goal, speed = scan_reference(elapsed, self.duration)
        goal, speed = goal*self.distance/.03, speed*self.distance/.03
        self.reference, vr = np.array([goal, self.normal_ref, 0.]), np.array([speed, vn, 0.])
        rotation = d.site_xmat[self.site].reshape(3, 3)
        if self.phase == 4:
            self.reference, vr, target_rotation = self.hold_x.copy(), np.zeros(3), self.hold_rotation
        else:
            target_rotation = ROTATION_GOAL
        er = orientation_error_world(rotation, target_rotation)
        force_s = K_TRANSLATION*(self.reference-x)+D_TRANSLATION*(vr-v)
        moment_w = K_ROTATION*er-D_ROTATION*omega
        bias = d.qfrc_bias[self.va].copy()  # Exact simulated bias; contact is not canceled.
        torque = bias+(self.jp.T @ (R_WS @ force_s)+self.jr.T @ moment_w)[self.va]
        if self.phase != 4 and np.any(np.abs(torque) > self.caps+1e-10):
            self.local_hold('failed', 'motor_saturation')
            self.reference, vr = self.hold_x.copy(), np.zeros(3)
            force_s = -D_TRANSLATION*v
            moment_w = -D_ROTATION*omega
            torque = bias+(self.jp.T @ (R_WS @ force_s)+self.jr.T @ moment_w)[self.va]
        d.ctrl[self.aid] = torque/self.gear
        mujoco.mj_forward(m, d)  # Same-state current command reaction/acceleration.
        reaction, active_now, _ = measure(m, d)
        self.force = float((R_WS.T @ reaction)[1])
        actual = d.qfrc_actuator[self.va].copy()
        np.testing.assert_allclose(actual, np.clip(torque, -self.caps, self.caps), atol=1e-10)
        np.testing.assert_allclose(d.qfrc_constraint, self.jp.T @ reaction, atol=1e-8)
        mujoco.mj_fullM(m, d, self.mass)
        residual = self.mass @ d.qacc+d.qfrc_bias-d.qfrc_actuator-d.qfrc_passive-d.qfrc_constraint
        assert np.max(np.abs(residual)) < 1e-8 and not np.any(d.qfrc_applied) and not np.any(d.xfrc_applied)
        self.metrics['max_dynamics_residual_Nm'] = max(self.metrics['max_dynamics_residual_Nm'], float(np.max(np.abs(residual))))
        self.metrics['saturated_samples'] += int(np.count_nonzero(np.abs(torque) > self.caps+1e-10))
        if self.status == 'active' and 5-1e-9 <= elapsed <= 5+2*self.duration+1e-9:
            for key, value in [('scan_tangent_error_m', abs(x[0]-goal)), ('scan_force_error_N', abs(self.force-TARGET)),
                               ('scan_binormal_error_m', abs(x[2])), ('scan_orientation_error_rad', np.linalg.norm(er))]:
                self.metrics[key] = max(self.metrics[key], float(value))
            self.metrics['scan_contact_all'] &= bool(active_now and self.force > .05)
        if self.status == 'active' and elapsed >= 11-1e-9:
            self.metrics['tail_tangent_error_m'] = max(self.metrics['tail_tangent_error_m'], float(abs(x[0]-goal)))
            self.metrics['tail_force_error_N'] = max(self.metrics['tail_force_error_N'], float(abs(self.force-TARGET)))
        if self.status == 'active' and elapsed >= 12-1e-9:
            a = self.metrics
            passed = (a['scan_contact_all'] and a['scan_tangent_error_m'] < .001 and a['scan_force_error_N'] < .05
                      and a['scan_binormal_error_m'] < .001 and a['scan_orientation_error_rad'] < .005
                      and a['tail_tangent_error_m'] < .0001 and a['tail_force_error_N'] < .05 and a['saturated_samples'] == 0)
            if passed:
                self.status, self.reason = 'succeeded', 'scan_qualified; local_force_hold_continues'
                self.event('succeeded')  # Completed task keeps bounded 2N force hold; no lease required.
            else:
                self.local_hold('failed', 'tracking_policy')
        age = time.monotonic()-self.issued if self.issued is not None else -1.
        self.rows.append(np.r_[d.time, elapsed, time.monotonic(), age, self.phase, self.reference, vr, x, v,
                               self.force, d.qpos[self.qa], d.qvel[self.va], torque, actual, d.qacc[self.va]])
        mujoco.mj_step(m, d)  # SOLE physics advancement; ROS never grants a tick.
        mujoco.mj_forward(m, d)  # Snapshot pose corresponds to updated qpos, not previous step.


class Mailboxes:
    def __init__(self):
        self.lock = threading.Lock()
        self.latest_reference = None  # Single overwrite slot: never queue an old reference backlog.
        self.lifecycle = queue.Queue(maxsize=8)
        self.state = None
        self.exit = threading.Event()

    def request(self, packet):
        op = packet.get('op')
        if op == 'state':
            with self.lock:
                return dict(self.state)
        if op == 'reference':
            with self.lock:
                self.latest_reference = dict(packet)
            return dict(queued=True)  # Enqueued ACK is NOT applied/accepted ACK; inspect state counters.
        if op not in ('begin', 'cancel', 'pause'):
            raise ValueError('unknown_operation; physics has no tick RPC')
        done, response = threading.Event(), {}
        self.lifecycle.put_nowait((packet, done, response))
        if not done.wait(2):
            raise TimeoutError('lifecycle application timeout; state uncertain')
        if 'error' in response:
            raise ValueError(response['error'])
        return response['state']


def serve(path, mail):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server:
        server.bind(str(path)); server.listen(4); server.settimeout(.2)
        while not mail.exit.is_set():
            try:
                conn, _ = server.accept()
            except socket.timeout:
                continue
            with conn:
                conn.settimeout(.3)  # A slow/partial client cannot block the physics owner.
                with conn.makefile('rwb') as stream:
                    try:
                        raw = stream.readline(16384)
                        if not raw.endswith(b'\n'): raise ValueError('bounded newline JSON required')
                        value = mail.request(json.loads(raw))
                        result = dict(ok=True, value=value)
                    except Exception as error:
                        result = dict(ok=False, reason=f'{type(error).__name__}: {error}')
                    try:
                        stream.write((json.dumps(result, allow_nan=False)+'\n').encode()); stream.flush()
                    except (OSError, ValueError):
                        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--socket', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('CONDA_DEFAULT_ENV') != 'mujoco' or Path(sys.prefix).name != 'mujoco':
        raise RuntimeError('require conda mujoco, no system/base Python fallback')
    if args.socket.exists(): raise RuntimeError('socket exists; do not replace another worker')
    args.output.mkdir(parents=True, exist_ok=True); args.socket.parent.mkdir(parents=True, exist_ok=True)
    controller, mail = Controller(), Mailboxes()
    mail.state = controller.snapshot()
    listener = threading.Thread(target=serve, args=(args.socket, mail), daemon=True); listener.start()
    print(f'worker_ready: {args.socket}; {sys.executable}', flush=True)
    try:
        while True:
            batch_started = time.monotonic()
            while not mail.lifecycle.empty():
                packet, done, response = mail.lifecycle.get_nowait()
                try: response['state'] = controller.lifecycle(packet)
                except Exception as error: response['error'] = str(error)
                with mail.lock: mail.state = controller.snapshot()
                done.set()
            with mail.lock:
                packet, mail.latest_reference = mail.latest_reference, None
            if packet is not None: controller.reference_packet(packet)
            if not controller.paused:
                for _ in range(10): controller.step()  # Fixed dt1ms, nominal 10ms wall batches.
            with mail.lock: mail.state = controller.snapshot()
            time.sleep(max(.0001, .01-(time.monotonic()-batch_started)))  # No catch-up; pacing is not realtime.
    except KeyboardInterrupt:
        pass
    finally:
        mail.exit.set(); listener.join(timeout=.5); args.socket.unlink(missing_ok=True)
        np.savetxt(args.output/'worker.csv', np.asarray(controller.rows), delimiter=',', comments='',
                   header='time_s,task_elapsed_s,wall_monotonic_s,command_age_s,phase,ref_t,ref_n,ref_b,ref_vt,ref_vn,ref_vb,t,n,b,vt,vn,vb,normal_N,'+
                          ','.join(f'{p}_{i}' for p in ('q','qvel','requested','motor','qacc') for i in range(6)))
        (args.output/'worker_result.json').write_text(json.dumps(dict(final=controller.snapshot(), events=controller.events,
            initialization=controller.initialization, dt_s=DT, trace_columns=48), indent=2)+'\n')


if __name__ == '__main__': main()
