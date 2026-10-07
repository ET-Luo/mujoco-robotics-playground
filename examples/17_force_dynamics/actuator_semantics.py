"""S16.1: position servo vs geared motor feedback under the same torque pulse."""

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np

TARGET = 0.3  # rad; positive rotation is right-handed about world +z.
KD = 1.0  # N*m*s/rad, active velocity feedback.
LIMIT = 1.5  # N*m, common joint actuator torque bound.
DT = 0.001  # s; fixed Euler step makes both feedback implementations explicit.
MODES = ("position_servo", "motor_feedback", "motor_open_loop")
COLUMNS = ("time_s", "q_rad", "qvel_rad_s", "ctrl", "actuator_force",
           "joint_actuator_torque_Nm", "external_joint_torque_Nm", "requested_joint_torque_Nm")


def build_model(mode, kp):
    """Compile identical mechanics, with either a gear-1 servo or gear-2 motor."""
    if mode == "position_servo":
        actuator = (f'<position name="drive" joint="hinge" gear="1" kp="{kp}" kv="{KD}" '
                    f'ctrllimited="false" forcelimited="true" forcerange="{-LIMIT} {LIMIT}"/>')
    else:
        # Scalar actuator force p is mapped to joint torque tau = gear * p.
        actuator = (f'<motor name="drive" joint="hinge" gear="2" ctrllimited="false" '
                    f'forcelimited="true" forcerange="{-LIMIT / 2} {LIMIT / 2}"/>')
    xml = f'''<mujoco model="actuator_semantics">
      <compiler angle="radian"/>
      <option timestep="{DT}" integrator="Euler" gravity="0 0 0"/>
      <worldbody><body name="rotor" pos="0 0 0.5">
        <inertial pos="0 0 0" mass="1" diaginertia="0.04 0.04 0.04"/>
        <joint name="hinge" type="hinge" axis="0 0 1" limited="false"
               damping="0.1" stiffness="0" frictionloss="0"/>
        <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"
              contype="0" conaffinity="0"/>
      </body></worldbody><actuator>{actuator}</actuator>
    </mujoco>'''
    # MjModel holds compiled mechanics/transmission/gains, not changing state.
    return mujoco.MjModel.from_xml_string(xml)


def command(model, data, mode, kp):
    """Return the requested joint torque; write one actuator's ctrl in place."""
    requested = kp * (TARGET - data.qpos[0]) - KD * data.qvel[0]
    if mode == "position_servo":
        data.ctrl[0] = TARGET  # gear-1 position servo: target angle [rad].
    elif mode == "motor_feedback":
        data.ctrl[0] = requested / model.actuator_gear[0, 0]
    else:
        requested = 0.0
        data.ctrl[0] = 0.0  # Open-loop torque, balanced only before the disturbance.
    return float(requested)


def static_checks(kp):
    """Equal-state torque checks separate semantics from accumulated integration error."""
    records = []
    for label, q, velocity in [("unsaturated", TARGET + 0.1 / kp, 0.1),
                               ("saturated", TARGET - 3.0 / kp, 0.0)]:
        for mode in MODES[:2]:
            model = build_model(mode, kp)
            assert (model.nq, model.nv, model.nu) == (1, 1, 1)
            assert model.actuator_trnid[0, 0] == model.joint("hinge").id
            gear = 1.0 if mode == "position_servo" else 2.0
            np.testing.assert_allclose(model.actuator_gear[0, 0], gear)
            np.testing.assert_allclose(model.actuator_gainprm[0, 0],
                                       kp if mode == "position_servo" else 1.0)
            np.testing.assert_allclose(model.actuator_biasprm[0, :3],
                                       [0, -kp, -KD] if mode == "position_servo" else [0, 0, 0])
            data = mujoco.MjData(model)  # Mutable qpos/qvel/ctrl and computed force arrays.
            data.qpos[0], data.qvel[0] = q, velocity
            requested = command(model, data, mode, kp)
            # Refresh computed actuator/generalized forces at this state, without advancing time.
            mujoco.mj_forward(model, data)
            expected = float(np.clip(requested, -LIMIT, LIMIT))
            np.testing.assert_allclose(data.qfrc_actuator[0], expected, atol=1e-12)
            np.testing.assert_allclose(data.qfrc_actuator[0],
                                       model.actuator_gear[0, 0] * data.actuator_force[0], atol=1e-12)
            assert data.time == 0.0
            records.append(dict(case=label, mode=mode, ctrl=float(data.ctrl[0]),
                                requested_torque_Nm=requested,
                                actuator_force=float(data.actuator_force[0]),
                                joint_torque_Nm=float(data.qfrc_actuator[0])))
    return records


def run_case(mode, kp, external_torque):
    model = build_model(mode, kp)
    data = mujoco.MjData(model)
    data.qpos[0] = TARGET  # Initialization only; never overwrite qpos during control.
    mujoco.mj_forward(model, data)
    rows = []
    # Each row is a PRE-step sample: state, command and measured forces share one instant.
    for step in range(4001):
        load = external_torque if 1000 <= step < 2000 else 0.0
        requested = command(model, data, mode, kp)
        data.qfrc_applied[0] = load  # External generalized hinge torque [N*m], not contact force.
        mujoco.mj_forward(model, data)
        rows.append([data.time, data.qpos[0], data.qvel[0], data.ctrl[0],
                     data.actuator_force[0], data.qfrc_actuator[0], load, requested])
        if step < 4000:
            mujoco.mj_step(model, data)  # Advances time/qpos/qvel in place; returns no state.
    samples = np.asarray(rows)
    if not np.isfinite(samples).all() or not np.isclose(data.time, 4.0, atol=1e-10):
        raise RuntimeError("nonfinite state or incorrect simulation duration")
    if np.max(np.abs(samples[:, 5])) > LIMIT + 1e-10:
        raise RuntimeError("joint actuator torque bound violated")
    loaded = samples[1800:2000]
    tail = samples[3500:]
    metrics = dict(loaded_mean_offset_rad=float(np.mean(loaded[:, 1] - TARGET)),
                   predicted_static_offset_rad=external_torque / kp if mode != MODES[2] else None,
                   final_q_rad=float(samples[-1, 1]),
                   tail_max_abs_error_rad=float(np.max(np.abs(tail[:, 1] - TARGET))),
                   tail_max_abs_speed_rad_s=float(np.max(np.abs(tail[:, 2]))),
                   peak_abs_actuator_torque_Nm=float(np.max(np.abs(samples[:, 5]))),
                   saturated_samples=int(np.count_nonzero(np.abs(samples[:, 7]) > LIMIT)))
    return samples, metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--kp", type=float, choices=(20.0, 40.0), default=20.0,
                        help="joint proportional gain [N*m/rad]; KD remains fixed")
    parser.add_argument("--external-torque", type=float, choices=(-0.8, 0.8), default=0.8,
                        help="synthetic external joint torque from 1 to 2 s [N*m]")
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    out = args.output or Path(f"tmp/s16_1_kp{args.kp:g}_load{args.external_torque:g}")
    out.mkdir(parents=True, exist_ok=True)
    result = dict(kp_Nm_per_rad=args.kp, kd_Nm_s_per_rad=KD, timestep_s=DT,
                  target_rad=TARGET, external_torque_Nm=args.external_torque,
                  joint_torque_limit_Nm=LIMIT, sample_convention="pre_step",
                  static_checks=static_checks(args.kp), modes={})
    traces = {}
    for mode in MODES:
        traces[mode], result["modes"][mode] = run_case(mode, args.kp, args.external_torque)
        np.savetxt(out / f"{mode}.csv", traces[mode], delimiter=",",
                   header=",".join(COLUMNS), comments="")
    difference = float(np.max(np.abs(traces[MODES[0]][:, 1:3] - traces[MODES[1]][:, 1:3])))
    result["servo_motor_max_state_difference"] = difference
    # Euler evaluates both feedback laws at the current state. Implicit actuator damping
    # and Python-computed damping would have different numerical treatment in implicitfast.
    if difference > 1e-9:
        raise RuntimeError("servo and explicit torque-feedback trajectories disagree")
    for mode in MODES[:2]:
        m = result["modes"][mode]
        if abs(m["loaded_mean_offset_rad"] - args.external_torque / args.kp) > 5e-4:
            raise RuntimeError("loaded equilibrium differs from torque balance")
        if m["tail_max_abs_error_rad"] > 1e-5 or m["tail_max_abs_speed_rad_s"] > 1e-4:
            raise RuntimeError("feedback did not recover over the tail window")
    if abs(result["modes"][MODES[2]]["final_q_rad"] - TARGET) < 0.1:
        raise RuntimeError("open-loop disturbance did not produce expected displacement")
    result["engineering_checks_passed"] = True
    (out / "results.json").write_text(json.dumps(result, indent=2) + "\n")
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    for mode, samples in traces.items():
        axes[0].plot(samples[:, 0], samples[:, 1] - TARGET, label=mode)
        axes[1].plot(samples[:, 0], samples[:, 1] - TARGET, label=mode)
        axes[2].plot(samples[:, 0], samples[:, 5], label=mode)
    axes[0].set_ylabel("Angle offset (rad), all modes")
    axes[1].set(ylabel="Angle offset (rad), feedback", ylim=(-0.06, 0.06))
    axes[2].plot(traces[MODES[0]][:, 0], traces[MODES[0]][:, 6], "k--", label="external joint torque")
    axes[2].set(xlabel="Simulation time (s)", ylabel="Joint torque (N m)")
    for ax in axes:
        ax.axvspan(1, 2, color="gray", alpha=0.12)
        ax.grid(True)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out / "response.png", dpi=140)
    plt.close(fig)
    print(json.dumps(result, indent=2))
    print(f"PASS: actuator semantics, torque limits, feedback equivalence and pulse recovery; output={out}")


if __name__ == "__main__":
    main()
