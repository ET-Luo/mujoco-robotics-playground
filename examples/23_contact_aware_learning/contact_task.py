"""S22.3 surface-following task: visible expert, bounded reference, 1kHz scorer."""
import numpy as np
import mujoco

from observation_contract import build_model, DT, observe

TASK = 'tip_surface_follow_v1'
OBS_SCHEMA = 'tip_force_goal_observation_v3'
ACTION_SCHEMA = 'actual_q_anchored_reference_delta_v1'
POLICY_DT = .02
STRIDE = 20
HORIZON = 150
READY_TIME = .6
FORCE_CAP = 5.
DELTA_CAP = np.array([.006, .008])
LOWER = np.array([-.025, -.042])
UPPER = np.array([.025, 0.])
MODES = {'state': list(range(6)), 'state_force': list(range(9)),
         'state_touch': list(range(6))+[9], 'full': list(range(10))}
OBS_NAMES = ['qx_m', 'qz_m', 'vx_m_s', 'vz_m_s', 'goal_x_m', 'target_force_N',
             'Fx_W_N', 'Fy_W_N', 'Fz_W_N', 'touch_N']
PHYSICS_NAMES = ['time_s', 'qx_m', 'qz_m', 'vx_m_s', 'vz_m_s', 'Fx_W_N', 'Fy_W_N',
                 'Fz_W_N', 'touch_N', 'actuator_x_N', 'actuator_z_N',
                 'ax_m_s2', 'az_m_s2', 'passive_x_N', 'passive_z_N']


def reset_parameters(seed, direction):
    rng = np.random.default_rng(seed)
    return dict(seed=int(seed), direction=int(direction), height=float(rng.uniform(-.003, .003)),
                qpos=[float(rng.uniform(-.002, .002)), float(rng.uniform(-.027, -.025))],
                qvel=[0., 0.], goal=[direction*.008, float(rng.uniform(.8, 1.6))])


def sense(model, data, goal):
    """Refresh current contacts/sensors, return observation and physics row; no time advance."""
    mujoco.mj_forward(model, data)
    force = np.zeros(3)
    tip, surface = model.geom('tip_geom').id, model.geom('surface').id
    for index in range(data.ncon):
        c = data.contact[index]
        if set(map(int, c.geom)) != {tip, surface}:
            continue
        raw = np.zeros(6)
        mujoco.mj_contactForce(model, data, index, raw)  # Write contact-frame wrench in place.
        sign = 1. if c.geom[1] == tip else -1.
        force += sign * c.frame.reshape(3, 3).T @ raw[:3]
    touch = float(data.sensor('touch').data[0])
    observation = np.r_[data.qpos.copy(), data.qvel.copy(), goal, force, touch]
    physics = np.r_[data.time, data.qpos.copy(), data.qvel.copy(), force, touch,
                    data.qfrc_actuator.copy(), data.qacc.copy(), data.qfrc_passive.copy()]
    return observation, physics


def expert(observation):
    q, velocity = observation[:2], observation[2:4]
    x_goal, target = observation[4:6]
    force = observation[8]  # Environment-on-tip normal force, positive world z [N].
    error = x_goal-q[0]
    dx = np.clip(.6*error-.05*velocity[0]+.5*force*np.tanh(error/.001)/200., -.006, .006) if force >= .05 else 0.
    if force < .05:
        dz = -.0003  # Search downward; source controller sees force, not hidden height.
    else:
        # kp=200 N/m, joint damping=2 Ns/m. Force error creates an upward correction.
        dz = (2*velocity[1]-target+.3*(force-target))/200.
    return np.array([dx, dz])


def score(physics, goal):
    """Full 1kHz latched failure, with fixed acquisition deadline and terminal tolerances."""
    time = physics[:, 0]
    force = physics[:, 7]
    after_ready = time >= READY_TIME-1e-10
    tail = time >= (HORIZON*POLICY_DT-.25)-1e-10
    bad = np.zeros(len(time), dtype=bool)
    bad |= force > FORCE_CAP
    bad |= after_ready & (force < .05)
    first = int(np.flatnonzero(bad)[0]) if bad.any() else None
    x_error = np.abs(physics[:, 1]-goal[0])
    f_error = np.abs(force-goal[1])
    speed = np.linalg.norm(physics[:, 3:5], axis=1)
    reached = bool(np.all(x_error[tail] < .001) and np.all(f_error[tail] < .25)
                   and np.all(speed[tail] < .002))
    return dict(success=first is None and reached, latched_failure=first is not None,
                first_failure_time_s=None if first is None else float(time[first]),
                first_failure_reason=None if first is None else ('overforce' if force[first] > FORCE_CAP else 'contact_not_ready_or_lost'),
                tail_reached=reached, peak_force_N=float(force.max()),
                contact_fraction_after_ready=float(np.mean(force[after_ready] >= .05)),
                tail_max_x_error_mm=float(x_error[tail].max()*1000),
                tail_max_force_error_N=float(f_error[tail].max()),
                tail_max_speed_mm_s=float(speed[tail].max()*1000))


def rollout(parameters, policy=expert):
    model = build_model(parameters['height'])  # Same sphere/plane/position-actuator physics as S22.1.
    model.opt.tolerance = 1e-12  # Consistent Newton-budget accuracy during friction transients.
    data = mujoco.MjData(model)
    data.qpos[:] = parameters['qpos']; data.qvel[:] = parameters['qvel']  # Initialization only.
    data.ctrl[:] = data.qpos
    goal = np.array(parameters['goal'])
    observation, _ = sense(model, data, goal)
    # Check optimized sensor extraction against the established observation implementation.
    _, checked = observe(model, data, goal)
    np.testing.assert_allclose(observation, checked, atol=1e-12)
    observations, held = [observation], [data.ctrl.copy()]
    requested, applied, refs, physics, clipped = [], [], [], [], []
    for _ in range(HORIZON):
        delta = np.asarray(policy(observations[-1]), dtype=float)
        if delta.shape != (2,) or not np.isfinite(delta).all():
            raise ValueError('policy must return finite delta-reference (2,) [m]')
        bounded = np.clip(delta, -DELTA_CAP, DELTA_CAP)
        ref = np.clip(data.qpos+bounded, LOWER, UPPER)
        requested.append(delta.copy()); applied.append(ref-data.qpos); refs.append(ref.copy())
        clipped.append(bool(np.max(np.abs(ref-data.qpos-delta)) > 1e-12))
        data.ctrl[:] = ref  # Absolute position reference [m], held for 20 physics steps.
        for _ in range(STRIDE):
            _, row = sense(model, data, goal)  # Current-command forces BEFORE integration, same timestamp.
            physics.append(row)
            mujoco.mj_step(model, data)  # In-place actual qpos/qvel/time integration; never copy future states.
        observation, _ = sense(model, data, goal)  # Previous held ctrl, before requesting the next action.
        observations.append(observation); held.append(data.ctrl.copy())
    _, row = sense(model, data, goal); physics.append(row)
    arrays = dict(observations=np.array(observations), requested_delta=np.array(requested),
                  actions=np.array(applied), references=np.array(refs), held_ctrl=np.array(held),
                  physics=np.array(physics), clipped=np.array(clipped))
    p = arrays['physics']
    assert arrays['observations'].shape == (HORIZON+1, 10) and p.shape == (HORIZON*STRIDE+1, 15)
    assert all(np.isfinite(v).all() for v in arrays.values())
    np.testing.assert_allclose(p[:, 0], np.arange(len(p))*DT, atol=1e-10, rtol=0)
    # Independent force sign/axes and Newton check: one .1kg body, two world-aligned slides.
    np.testing.assert_allclose(.1*p[:, 11:13], p[:, 9:11]+p[:, 13:15]+p[:, [5, 7]], atol=1e-8)
    np.testing.assert_allclose(p[:, 7], p[:, 8], atol=1e-9)  # One horizontal normal contact.
    np.testing.assert_array_equal(arrays['held_ctrl'][1:], arrays['references'])
    np.testing.assert_allclose(arrays['actions'], arrays['references']-arrays['observations'][:-1, :2], atol=1e-12)
    result = score(p, goal)
    result['clipped_ticks'] = int(np.sum(clipped))
    return arrays, result
