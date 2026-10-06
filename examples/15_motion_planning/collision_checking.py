"""S14.1: isolated configuration queries and phase-specific contact policy."""
import argparse
import json
from pathlib import Path

import mujoco
import numpy as np

ROBOT = {'base', 'palm', 'left_pad', 'right_pad'}
ENVIRONMENT = {'ground', 'obstacle'}
PHASES = ('approach', 'grasp', 'transfer', 'place')


def build_model():
    # Small Cartesian teaching fixture; not a UR5e model or grasp simulation.
    return mujoco.MjModel.from_xml_string('''
    <mujoco model="collision_lesson">
      <compiler angle="radian"/>
      <option gravity="0 0 0"/>
      <default><geom margin="0.01" gap="0.01"/></default>
      <worldbody>
        <geom name="ground" type="plane" size="1 1 .01"/>
        <geom name="obstacle" type="sphere" pos=".14 0 .5" size=".01"/>
        <body name="base_body" pos="-.4 0 .5">
          <geom name="base" type="sphere" size=".04"/>
        </body>
        <body name="tool_body" pos="0 0 .5">
          <joint name="x" type="slide" axis="1 0 0" range="-.5 .5"/>
          <joint name="y" type="slide" axis="0 1 0" range="-.5 .5"/>
          <joint name="z" type="slide" axis="0 0 1" range="-.49 .5"/>
          <joint name="yaw" type="hinge" axis="0 0 1" range="-3.14 3.14"/>
          <geom name="palm" type="sphere" pos="-.12 0 0" size=".02"/>
          <geom name="left_pad" type="sphere" pos=".06 .04 0" size=".015"/>
          <geom name="right_pad" type="sphere" pos=".06 -.04 0" size=".015"/>
          <site name="tool"/>
        </body>
        <body name="object_body" pos=".06 0 .5">
          <freejoint name="object_free"/>
          <geom name="object" type="sphere" size=".03"/>
        </body>
      </worldbody>
    </mujoco>''')


def allowed_pair(pair, phase):
    """Allow only named pairs; never whitelist a whole body/category."""
    pair = frozenset(pair)
    pads = [frozenset(('object', pad)) for pad in ('left_pad', 'right_pad')]
    return ((phase in ('grasp', 'transfer', 'place') and pair in pads)
            or (phase in ('approach', 'grasp', 'place')
                and pair == frozenset(('object', 'ground'))))


def category(pair):
    names = set(pair)
    if names <= ROBOT:
        return 'self'
    if 'object' in names:
        return 'object_environment' if names & ENVIRONMENT else 'robot_object'
    return 'robot_environment'


def check_configuration(model, snapshot, q, phase, held=False, allowed_depth_m=.006):
    """Return a report; mutate a new MjData only, never snapshot or model."""
    q = np.asarray(q, dtype=float)
    if phase not in PHASES:
        raise ValueError('unknown phase')
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError('q must be finite shape (4,), slides in m, yaw in rad')
    if not np.isfinite(allowed_depth_m) or not 0 <= allowed_depth_m <= .01:
        raise ValueError('allowed depth must be in [0, .01] m')
    if held != (phase in ('transfer', 'place')):
        raise ValueError('held must match transfer/place phase')
    if np.any(q < model.jnt_range[:4, 0]) or np.any(q > model.jnt_range[:4, 1]):
        return {'valid': False, 'reason': 'joint_limits', 'contacts': []}

    # MjData(model) allocates writable state/caches for this shared read-only model.
    query = mujoco.MjData(model)
    query.qpos[:] = snapshot.qpos
    query.qpos[:4] = q
    # mj_forward(model, data) returns None, updates FK/contact/dynamics in-place,
    # and does not integrate qpos or advance time.
    mujoco.mj_forward(model, query)
    if held:
        # Fixed T_GO: object -> gripper; known geometric attachment hypothesis.
        # T_WO(q) = T_WG(q) @ T_GO, with translation .06 m along tool +x.
        site = model.site('tool').id
        rotation = query.site_xmat[site].reshape(3, 3)
        address = model.jnt_qposadr[model.joint('object_free').id]
        query.qpos[address:address+3] = query.site_xpos[site] + rotation @ [.06, 0, 0]
        # mju_mat2Quat writes quaternion (w,x,y,z) into the supplied (4,) buffer.
        mujoco.mju_mat2Quat(query.qpos[address+3:address+7], rotation.ravel())
        mujoco.mj_forward(model, query)

    contacts = []
    for contact in query.contact:
        pair = sorted((model.geom(contact.geom1).name, model.geom(contact.geom2).name))
        distance = float(contact.dist)  # m: positive separation, negative penetration.
        permitted = allowed_pair(pair, phase)
        # Positive-margin records are diagnostics, not touching/penetrating pairs.
        violation = distance <= 0 and (not permitted or distance < -allowed_depth_m)
        contacts.append(dict(pair=pair, category=category(pair), distance_m=distance,
                             permitted_pair=permitted, violation=violation))
    return dict(valid=not any(c['violation'] for c in contacts), reason='geometry',
                contacts=contacts, object_position_m=query.xpos[model.body('object_body').id].tolist(),
                query_time_s=float(query.time))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--allowed-depth-m', type=float, default=.006)
    args = parser.parse_args()
    if not np.isfinite(args.allowed_depth_m) or not 0 <= args.allowed_depth_m <= .01:
        parser.error('--allowed-depth-m must be finite and in [0, .01]')
    model = build_model()
    live = mujoco.MjData(model)
    live.time = 1.25
    live.qvel[:] = .123
    mujoco.mj_forward(model, live)
    before = (live.qpos.copy(), live.qvel.copy(), live.time, live.xpos.copy())
    cases = [
        ('clear', [0, .2, 0, 0], 'approach', False),
        ('pads_approach', [0, 0, 0, 0], 'approach', False),
        ('pads_grasp', [0, 0, 0, 0], 'grasp', False),
        ('self_collision', [-.28, 0, 0, 0], 'approach', False),
        ('robot_obstacle', [.26, 0, 0, 0], 'approach', False),
        ('held_clear', [0, .2, 0, 0], 'transfer', True),
        ('held_obstacle', [.06, 0, 0, 0], 'transfer', True),
        ('ground_transfer', [0, .2, -.4705, 0], 'transfer', True),
        ('ground_place', [0, .2, -.4705, 0], 'place', True),
        ('deep_place', [0, .2, -.48, 0], 'place', True),
        ('held_rotated', [0, .2, 0, 1.57], 'transfer', True),
        ('joint_limit', [.6, 0, 0, 0], 'approach', False),
        ('near_obstacle', [.035, 0, 0, 0], 'transfer', True),
    ]
    results = []
    for name, q, phase, held in cases:
        result = check_configuration(model, live, q, phase, held, args.allowed_depth_m)
        result.update(name=name, q=q, phase=phase, held=held)
        results.append(result)
        pairs = [(c['pair'], round(c['distance_m'], 6)) for c in result['contacts']]
        print(f"{name:18s} valid={result['valid']} contacts={pairs}")
    # Engineering invariants and meaningful known-geometry checks.
    assert np.array_equal(live.qpos, before[0])
    assert np.array_equal(live.qvel, before[1]) and live.time == before[2]
    assert np.array_equal(live.xpos, before[3])
    by_name = {r['name']: r for r in results}
    for name in ('pads_approach', 'self_collision', 'robot_obstacle',
                 'held_obstacle', 'ground_transfer', 'deep_place', 'joint_limit'):
        assert not by_name[name]['valid'], name
    for name in ('pads_grasp', 'held_clear', 'ground_place', 'held_rotated', 'near_obstacle'):
        assert by_name[name]['valid'] == (args.allowed_depth_m >= max(-c['distance_m'] for c in by_name[name]['contacts'] if c['permitted_pair'])), name
    assert by_name['clear']['valid']
    assert any(c['distance_m'] > 0 for c in by_name['near_obstacle']['contacts'])
    assert all(r.get('query_time_s', 0) == 0 for r in results)
    rotated = by_name['held_rotated']['object_position_m']
    np.testing.assert_allclose(rotated, [.06*np.cos(1.57), .2+.06*np.sin(1.57), .5])
    out = Path('tmp') / f's14_1_collision_depth{args.allowed_depth_m:g}'
    out.mkdir(parents=True, exist_ok=True)
    (out / 'results.json').write_text(json.dumps(results, indent=2)+'\n')
    print(f'PASS: isolated state, collision cases, held transform; output={out}')


if __name__ == '__main__':
    main()
