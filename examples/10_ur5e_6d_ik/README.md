# One-step UR5e 6D inverse kinematics

S10.4 forms a world-frame 6D pose error, solves one full-Jacobian linear system, applies the resulting
joint increment directly to `qpos`, and recomputes the nonlinear pose error. It deliberately has no loop,
damping, actuator command, or dynamics.

Run the verified baseline from the repository root:

```bash
python examples/10_ur5e_6d_ik/main.py
```

For the learner's modification, enlarge the same reachable target:

```bash
python examples/10_ur5e_6d_ik/main.py --target-scale 5
```

The target pose is generated once from a small known joint offset so the experiment is reproducible.
The IK calculation itself only receives the target position and rotation, not that generating offset.
Compare the final position/orientation residuals: one-step IK should improve both, while the larger target
usually leaves a larger nonlinear residual because a fixed Jacobian is only a local first-order model.

Engineering validation on 2026-10-03 used WSL2, conda `mujoco`, Python 3.12.14, and MuJoCo 3.14.0.
Both commands exited 0. The baseline reduced position/orientation norms from about
`1.406 mm / 4.529 mrad` to `1.922 um / 1.124 urad`. At scale 5, they fell from about
`7.022 mm / 22.663 mrad` to `47.974 um / 27.950 urad`. The larger residual is expected from applying
one fixed local linearization to a larger target. These assistant runs do not count as learner Run/Modify.

## S10.5 Damped Least Squares

[`dls_comparison.py`](dls_comparison.py) moves `wrist_2_joint` near its wrist singularity, constructs a
small pose error along the weakest left-singular direction, and compares undamped least-squares with
DLS at `lambda=1e-3` and `1e-2`:

```bash
python examples/10_ur5e_6d_ik/dls_comparison.py
python examples/10_ur5e_6d_ik/dls_comparison.py --wrist-angle 0.05
```

The second command is the learner modification: moving farther from the singular reference should lower
the condition number and reduce undamped inverse amplification. The experiment uses the numerical
convention `[position_m; rotation_rad]`; changing that scaling changes singular values and damping meaning.

## S10.6 Iterative 6D IK

[`iterative_ik.py`](iterative_ik.py) repeats DLS while recomputing FK, error, and Jacobian every update.
It has separate position/orientation tolerances, an update limit, a maximum joint-step component, joint
position checks, finite-value checks, and explicit terminal statuses:

```bash
python examples/10_ur5e_6d_ik/iterative_ik.py
python examples/10_ur5e_6d_ik/iterative_ik.py --scenario difficult --max-joint-step 0.02
```

The second command is the learner modification. A smaller per-update step should usually require more
accepted updates while preserving bounded commands. `UPDATE_LIMIT` means the configured solver budget
was exhausted; it is not a general mathematical proof that the target is unreachable.

## S10.7 Joint velocity limits

[`velocity_limited_ik.py`](velocity_limited_ik.py) converts each DLS increment into a geometric velocity
command using `qdot_command = delta_q / control_dt` and enforces
`|delta_q_i| <= qdot_max_i * control_dt` with one common scale:

```bash
python examples/10_ur5e_6d_ik/velocity_limited_ik.py
python examples/10_ur5e_6d_ik/velocity_limited_ik.py --control-dt 0.01
```

The second command is the learner modification. Halving `control_dt` halves every per-update angle
limit, so the same geometric path should require roughly more updates while respecting the same rad/s
limit. The script directly integrates `qpos` and calls `mj_forward`; its `qdot_command` is not MuJoCo's
actual `data.qvel`, because no actuator command or `mj_step` executes that velocity.
