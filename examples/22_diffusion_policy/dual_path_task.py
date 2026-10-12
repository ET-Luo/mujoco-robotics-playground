"""B track: XY two-route demonstrations with a geometric forbidden disk.

The disk is a task constraint, NOT a contact geom. Collision is latched from
actual 1kHz motion segments; MuJoCo still integrates the unconstrained XY stage.
"""
from pathlib import Path
import sys

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / '20_learning_from_demonstrations'))
import episode_contract as env

TASK = 'xy_two_route_disk_v1'
RADIUS = .010  # m; forbidden radius for the tracked XY point, not old visual spheres.
HORIZON = 150  # 3s: three 0.6s cubic segments + 1.2s goal hold.
SEGMENT_TICKS = 30


def reset(seed):
    rng = np.random.default_rng(seed)
    start = np.array([rng.uniform(-.032, -.028), 0.])
    goal = np.array([rng.uniform(.028, .032), 0.])
    return np.r_[start, np.zeros(2), goal]


def references(initial, side):
    """Scripted absolute position references (T,2), [m] in world XY.

    Route side is a demonstration choice hidden from the learner observation.
    Phase is internal to the scripted expert, not an extra observed feature.
    """
    if side not in (-1, 1):
        raise ValueError('side must be -1 or +1')
    points = np.array([initial[:2], [-.018, side*.026], [.018, side*.026], initial[4:6]])
    values = []
    for a, b in zip(points[:-1], points[1:]):
        u = np.arange(1, SEGMENT_TICKS+1)/SEGMENT_TICKS
        smooth = 3*u**2 - 2*u**3
        values.extend(a[None] + smooth[:,None]*(b-a))
    values.extend(np.tile(initial[4:6], (HORIZON-len(values), 1)))
    return np.asarray(values)


def segment_clearance(a, b):
    """Signed distance [m] from segment to origin-centered forbidden disk."""
    a, b = np.asarray(a), np.asarray(b)
    delta = b-a
    length2 = np.sum(delta*delta, axis=-1)
    fraction = np.divide(-np.sum(a*delta, axis=-1), length2,
                         out=np.zeros_like(length2, dtype=float), where=length2>0)
    closest = a + np.clip(fraction, 0., 1.)[...,None]*delta
    return np.linalg.norm(closest, axis=-1)-RADIUS


def clearance(points):
    points = np.asarray(points)
    if points.ndim != 2 or points.shape[1] != 2 or len(points)<2 or not np.isfinite(points).all():
        raise ValueError('expected finite XY polyline with at least two points')
    return segment_clearance(points[:-1], points[1:])


def rollout(initial, scripted=None, policy=None, norm=None, kind=None, execute=1):
    if execute not in (1, 5):
        raise ValueError('execute must be 1 or 5')
    if scripted is not None and np.asarray(scripted).shape != (HORIZON, 2):
        raise ValueError('scripted reference shape mismatch')
    if scripted is None and kind not in ('bc', 'act'):
        raise ValueError('expected scripted plan or BC/ACT policy')
    if scripted is None and kind == 'bc' and execute != 1:
        raise ValueError('single-action BC requires execute=1')
    model = mujoco.MjModel.from_xml_string(env.XML)  # Existing compiled stage/masses/motors.
    data = mujoco.MjData(model)  # Independent mutable physical state.
    data.qpos[:] = initial[:2]; data.qvel[:] = initial[2:4]  # Reset only.
    goal = initial[4:6].copy()
    mujoco.mj_forward(model, data)  # Refresh derived quantities in place, no time advance.
    observations = [env.observe(data, goal)]
    actions, physics_xy, plans, anchors, offsets, prediction_ticks = [], [data.qpos.copy()], [], [], [], []
    calls = 0
    for tick in range(HORIZON):
        if scripted is not None:
            action = scripted[tick]
        else:
            import torch
            if tick % execute == 0:
                x = (observations[-1]-norm['observations_mean'])/norm['observations_scale']
                with torch.inference_mode():
                    observation = torch.tensor(x[None], dtype=torch.float32)
                    value = policy.infer(observation)[0] if kind=='act' else policy(observation)[0][None]
                    plan = value.numpy()*norm['actions_scale']+norm['actions_mean']
                plans.append(plan.copy()); prediction_ticks.append(tick); calls += 1
            offset = tick % execute
            action = plan[offset]
            anchors.append(tick-offset); offsets.append(offset)
        if not np.isfinite(action).all():
            raise ValueError('nonfinite reference')
        actions.append(action.copy())
        for _ in range(env.STRIDE):
            # Reference is not ctrl: recompute bounded impedance force at 1kHz.
            requested = env.K*(action-data.qpos)-env.D*data.qvel
            data.ctrl[:] = requested/env.GEAR
            mujoco.mj_step(model, data)  # In-place qpos/qvel/time integration.
            physics_xy.append(data.qpos.copy())
        observations.append(env.observe(data, goal))
    o = np.asarray(observations); physical = np.asarray(physics_xy)
    signed = clearance(physical)
    hit = np.flatnonzero(signed<=0.)
    error = np.linalg.norm(o[-10:,:2]-goal,axis=1)
    speed = np.linalg.norm(o[-10:,2:4],axis=1)
    tail = bool(np.all(error<.001) and np.all(speed<.002))
    crossed = np.flatnonzero(physical[:,0]>=0.)
    side = 'unresolved'
    if len(crossed):
        y = physical[crossed[0],1]
        side = 'upper' if y>RADIUS else 'lower' if y<-RADIUS else 'center'
    assert np.isclose(data.time, HORIZON*env.POLICY_DT)
    trace = dict(observations=o, actions=np.asarray(actions), physics_xy=physical,
                 time_s=np.arange(HORIZON+1)*env.POLICY_DT)
    if scripted is None:
        trace.update(predicted_chunks=np.asarray(plans), prediction_timestep=np.asarray(prediction_ticks),
                     plan_anchor=np.asarray(anchors), chunk_offset=np.asarray(offsets))
    metrics = dict(success=bool(tail and not len(hit)), collision=bool(len(hit)), tail_goal_reached=tail,
                   minimum_clearance_mm=1000*float(signed.min()),
                   first_collision_segment_start_s=float(hit[0]*env.DT) if len(hit) else None,
                   final_error_mm=1000*float(np.linalg.norm(o[-1,:2]-goal)),
                   tail_max_speed_mm_s=1000*float(speed.max()), route=side,
                   outcome='collision' if len(hit) else 'success' if tail else 'time_limit_criterion_not_met',
                   physics_steps=HORIZON*env.STRIDE, prediction_calls=calls,
                   simulation_duration_s=HORIZON*env.POLICY_DT, truncated=True)
    return trace, metrics


def geometry_probes():
    np.testing.assert_allclose(segment_clearance(np.array([-.02,0.]),np.array([.02,0.])), -RADIUS)
    np.testing.assert_allclose(segment_clearance(np.array([-.02,.02]),np.array([.02,.02])), .01)
    np.testing.assert_allclose(segment_clearance(np.array([.02,0.]),np.array([.02,0.])), .01)
    np.testing.assert_allclose(segment_clearance(np.array([-.02,RADIUS]),np.array([.02,RADIUS])), 0.)
    return dict(tunneling_detected=True, tangent_is_collision=True, stationary_segment=True)
