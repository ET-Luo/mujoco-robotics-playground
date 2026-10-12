"""S22.6: causal 50 Hz observation/action loop, 1 kHz free-object physics/scorer."""
from pathlib import Path
import sys

FOUNDATIONS = Path(__file__).resolve().parents[1] / '19_teleoperation_dexterous'
sys.path.insert(0, str(FOUNDATIONS))
import grasp_stability as limits
from hand_fixture import CLOSED, JOINTS, KP, KV, TORQUE_CAP, mapping
from multi_contact_grasp import build_model, object_contacts, MASS
import mujoco
import numpy as np

TASK = 'three_finger_causal_hold_v2'
OBS_SCHEMA = 'q6_v6_object_pose7_world_velocity6_normal3_previous_ctrl6_time1_phase3_v1'
ACTION_SCHEMA = 'absolute_joint_position_rad_box_and_rate_v1'
DT, POLICY_STEPS, HORIZON = .001, 20, 200
RATE_CAP = .02  # rad per 20ms action tick, componentwise; absolute target remains the action.
UPPER = np.tile([.9, 1.2], 3)
OBS_COLUMNS = ([f'q_{j}_rad' for j in JOINTS] + [f'qvel_{j}_rad_s' for j in JOINTS]
               + [f'object_{a}_W_m' for a in 'xyz'] + ['qw', 'qx', 'qy', 'qz']
               + [f'omega_{a}_W_rad_s' for a in 'xyz'] + [f'velocity_{a}_W_m_s' for a in 'xyz']
               + [f'normal_f{i}_N' for i in range(3)] + [f'previous_ctrl_{j}_rad' for j in JOINTS]
               + ['time_s', 'phase_close', 'phase_hold', 'phase_fault'])
PHYSICS_COLUMNS = (['time_s', 'phase', 'ready'] + [f'normal_f{i}_N' for i in range(3)]
                   + [f'contact_F{a}_W_N' for a in 'xyz'] + [f'contact_T{a}_W_Nm' for a in 'xyz']
                   + [f'applied_F{a}_W_N' for a in 'xyz'] + [f'applied_T{a}_W_Nm' for a in 'xyz']
                   + ['drift_m', 'angle_rad', 'slip_m_s', 'support_N', 'drop']
                   + [f'omega_{a}_W_rad_s' for a in 'xyz'] + [f'velocity_{a}_W_m_s' for a in 'xyz']
                   + [f'actual_torque_{j}_Nm' for j in JOINTS] + ['load_N'])


def reset_parameters(seed):
    """Same small reset distribution for all split groups; no mass/friction domain sweep."""
    rng = np.random.default_rng(seed)
    return dict(seed=int(seed), finger_q_rad=rng.uniform(0, .003, 6).tolist(),
                object_offset_W_m=rng.uniform(-.0003, .0003, 3).tolist(),
                yaw_rad=float(rng.uniform(-.03, .03)))


def bounded_action(raw, previous):
    raw = np.asarray(raw, dtype=float)
    previous = np.asarray(previous, dtype=float)
    if raw.shape != (6,) or previous.shape != (6,) or not np.isfinite(raw).all() or not np.isfinite(previous).all():
        raise ValueError('finite six-dimensional action and previous reference required')
    box = np.clip(raw, 0., UPPER)
    return previous + np.clip(box-previous, -RATE_CAP, RATE_CAP)


def failures_at(normal, drift, angle, slip, support, drop):
    return dict(contact_loss=bool(np.any(normal <= limits.KEEP_CONTACT)),
                drift=bool(drift >= limits.DRIFT_LIMIT), rotation=bool(angle >= limits.ANGLE_LIMIT),
                slip=bool(slip >= limits.SLIP_LIMIT), nonfinger_support=bool(support >= 1e-8),
                downward_drop=bool(drop))


def score(physics):
    """Pure full-trajectory scorer, shared by experts and every learned-policy attempt."""
    c = {key: physics[:, i] for i, key in enumerate(PHYSICS_COLUMNS)}
    normal = physics[:, 3:6]
    failure_times = {}
    for i in range(2000, 4001):
        checks = failures_at(normal[i], c['drift_m'][i], c['angle_rad'][i], c['slip_m_s'][i],
                             c['support_N'][i], c['drop'][i])
        for name, failed in checks.items():
            if failed:
                failure_times.setdefault(name, float(c['time_s'][i]))
    timeout = np.flatnonzero(c['phase'] == 2)
    if len(timeout):
        failure_times['close_timeout'] = float(c['time_s'][timeout[0]])
    angular = physics[:, [PHYSICS_COLUMNS.index(f'omega_{a}_W_rad_s') for a in 'xyz']]
    linear = physics[:, [PHYSICS_COLUMNS.index(f'velocity_{a}_W_m_s') for a in 'xyz']]
    tail_linear = float(np.linalg.norm(linear[3750:], axis=1).max())
    tail_angular = float(np.linalg.norm(angular[3750:], axis=1).max())
    if tail_linear >= limits.LINEAR_TAIL_LIMIT or tail_angular >= limits.ANGULAR_TAIL_LIMIT:
        failure_times['tail_not_recovered'] = 3.75
    ready = bool(c['ready'][2500])
    pulse = physics[:, 12:18].copy()
    # Remove the object-only constant load to distinguish a zero-pulse baseline.
    pulse[:, 2] += c['load_N']
    return dict(success=bool(ready and not failure_times), ready=ready,
                pulse_delivered=bool(np.any(np.abs(pulse) > 1e-12)), failure_first_times_s=failure_times,
                max_drift_m=float(c['drift_m'][2000:].max()), max_angle_rad=float(c['angle_rad'][2000:].max()),
                max_slip_m_s=float(c['slip_m_s'][2000:].max()), downward_drop=bool(np.any(c['drop'][2000:])),
                tail_linear_speed_m_s=tail_linear, tail_angular_speed_rad_s=tail_angular)


def rollout(parameters, pulse='baseline', pulse_scale=1., policy=None, replay_actions=None):
    """Observe under PREVIOUS ctrl, decide at 50Hz, solve/score and integrate at 1kHz."""
    limits.pulse_wrench(0., pulse, pulse_scale)
    if replay_actions is not None and (policy is not None or np.shape(replay_actions) != (HORIZON, 6)):
        raise ValueError('replay requires exactly 200 actions and no learned policy')
    model, _ = build_model('three_finger')
    # Uniform tighter solver for every new expert/learner trial, not a relaxed force check.
    model.opt.tolerance = 1e-14; model.opt.iterations = 200
    data = mujoco.MjData(model)
    _, qa, va, aa = mapping(model)
    body = model.body('object').id; joint = model.joint('object_free').id
    oq, ov = int(model.jnt_qposadr[joint]), int(model.jnt_dofadr[joint])
    data.qpos[qa] = parameters['finger_q_rad']
    data.qpos[oq:oq+3] += parameters['object_offset_W_m']
    yaw = parameters['yaw_rad']; data.qpos[oq+3:oq+7] = [np.cos(yaw/2), 0., 0., np.sin(yaw/2)]
    if not np.isfinite(data.qpos).all():
        raise ValueError('finite reset required')
    # qpos writes are initial reset only; no object or finger state is overwritten during rollout.
    initial_qpos = data.qpos.copy(); initial_qvel = data.qvel.copy()
    phase, count, ready, gate_time = 0, 0, False, None
    failures = {}; anchor_p = anchor_q = None
    observations, previous_ctrl, actions, raw_actions, plan_s = [], [], [], [], []
    physics, qpos, qvel, qacc, ctrl = [], [], [], [], []
    velocity = np.zeros(6)
    max_newton = 0.; cap_count = 0
    for step in range(4001):
        now = step*DT
        if step == 2500:
            ready = bool(phase == 1 and not failures)
        applied = limits.pulse_wrench(now, pulse, pulse_scale) if ready else np.zeros(6)
        load = .1 if step >= 2000 and phase == 1 else 0.
        applied[2] -= load
        data.xfrc_applied[:] = 0.; data.xfrc_applied[body] = applied  # world [F,T] at COM.
        mujoco.mj_forward(model, data)  # Previous reference's current contact solve, no time advance.
        _, pre_normal, _, _, _, _, _ = object_contacts(model, data, body)
        if phase == 0:
            count = count+1 if np.all(pre_normal > limits.CONTACT_GATE) else 0
            if count >= 50:
                phase = 1; gate_time = now
            elif step >= 2000:
                phase = 2; failures.setdefault('close_timeout', now)
        if step % POLICY_STEPS == 0:
            previous = data.ctrl[aa].copy()
            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, body, velocity, 0)
            # 38 inputs: past executed reference and observable monitor phase supply expert memory.
            observation = np.r_[data.qpos[qa], data.qvel[va], data.xipos[body], data.xquat[body],
                                velocity, pre_normal, previous, now, np.eye(3)[phase]]
            observations.append(observation); previous_ctrl.append(previous)
            if step < 4000:
                if replay_actions is not None:
                    raw = replay_actions[step//POLICY_STEPS].copy()
                elif policy is None:
                    raw = CLOSED*min(now, 1.) if phase == 0 else previous.copy()
                else:
                    import time
                    started = time.perf_counter(); raw = policy(observation.copy())
                    plan_s.append(time.perf_counter()-started)
                action = bounded_action(raw, previous)
                cap_count += int(np.any(np.abs(action-raw) > 1e-12))
                raw_actions.append(raw); actions.append(action)
                data.ctrl[aa] = action  # Named gear-1 position-servo references [rad], not torque.
                mujoco.mj_forward(model, data)  # Post-action solve for physics/scorer; NOT policy input.
        _, normal, force, moment, slip, support, _ = object_contacts(model, data, body)
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, body, velocity, 0)
        requested = KP*(data.ctrl[aa]-data.qpos[qa])-KV*data.qvel[va]
        actual = data.actuator_force[aa].copy()
        np.testing.assert_allclose(actual, np.clip(requested, -TORQUE_CAP, TORQUE_CAP), atol=1e-10)
        residual = float(np.max(np.abs(force+applied[:3]-MASS*data.qacc[ov:ov+3])))
        max_newton = max(max_newton, residual)
        assert residual < 1e-6, residual
        if step == 2000:
            anchor_p = data.xipos[body].copy(); anchor_q = data.xquat[body].copy()
        drift = angle = 0.; drop = False
        if anchor_p is not None:
            drift = float(np.linalg.norm(data.xipos[body]-anchor_p))
            angle = float(2*np.arccos(np.clip(abs(data.xquat[body]@anchor_q), 0, 1)))
            drop = bool(data.xipos[body, 2] < anchor_p[2]-limits.DROP_DISTANCE)
            for name, failed in failures_at(normal, drift, angle, slip, support, drop).items():
                if failed: failures.setdefault(name, now)
        physics.append(np.r_[now, phase, ready, normal, force, moment, applied,
                             drift, angle, slip, support, drop, velocity, actual, load])
        qpos.append(data.qpos.copy()); qvel.append(data.qvel.copy()); qacc.append(data.qacc.copy())
        ctrl.append(data.ctrl[aa].copy())
        if step < 4000:
            mujoco.mj_step(model, data)  # Actual finger/free-object integration; one millisecond.
    arrays = dict(observations=np.asarray(observations), previous_ctrl=np.asarray(previous_ctrl),
                  actions=np.asarray(actions), raw_actions=np.asarray(raw_actions), physics=np.asarray(physics),
                  qpos=np.asarray(qpos), qvel=np.asarray(qvel), qacc=np.asarray(qacc), ctrl=np.asarray(ctrl),
                  initial_qpos=initial_qpos, initial_qvel=initial_qvel, plan_s=np.asarray(plan_s))
    assert arrays['observations'].shape == (201, 38) and arrays['actions'].shape == (200, 6)
    assert arrays['physics'].shape == (4001, len(PHYSICS_COLUMNS))
    assert all(np.isfinite(v).all() for v in arrays.values())
    result = score(arrays['physics'])
    result.update(parameters=parameters, pulse=pulse, pulse_scale=pulse_scale, gate_time_s=gate_time,
                  physics_steps=4000, action_cap_ticks=cap_count, max_Newton_residual_N=max_newton)
    return arrays, result
