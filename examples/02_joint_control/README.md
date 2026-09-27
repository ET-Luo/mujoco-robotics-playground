# Joint control

Status: Stage 2 uses the existing UR5e example; this directory contains lesson navigation.

Goal: Understand how an actuator maps ctrl into joint actuation.

S2.1–S2.4 learning exercises are complete. The assistant validated both
+0.02/+0.05 rad runs from home; the learner correctly compared final errors,
identified motion of another joint, and understood that valid targets do not
guarantee zero error. This does not imply independent controller implementation.

After activating and verifying mujoco, run from the repository root:

```bash
python examples/02_ur5e_basics/main.py --headless --steps 1000 --delta 0.02 --plot tmp/response_002.png
python examples/02_ur5e_basics/main.py --headless --steps 1000 --delta 0.05 --plot tmp/response_005.png
```

Compare the printed final error and all joint angle changes. Each command resets
to home independently. Out-of-range targets are rejected before stepping.

S2.1 parameter reading is complete for the existing UR5e `shoulder_pan` actuator.
See [the lesson note](../../docs/03_joint_control.md). On 2026-09-27 the learner
answered the units, input range, and gear target-conversion exercises. No controller
or simulation code is added for this step. Task checkboxes live in the root README.

S2.2 is complete: the learner predicted the home and pre-step states, supplied
the 2-second output from the existing [UR5e example](../02_ur5e_basics/main.py),
and explained the result, correcting the assumption that unchanged targets lock
other joints. No new code was needed. The assistant did not rerun the simulation
or validate GUI operation.

S2.3 records simulation time, target, and actual angle at t=0 and after each step.
After activating and verifying mujoco, run from the repository root:

```bash
python examples/02_ur5e_basics/main.py --headless --steps 1000 --plot tmp/shoulder_pan_response.png
```

This saves a PNG and a matching CSV with 1,001 rows to the ignored tmp directory.
The plot option supports headless mode only. Matplotlib is already a dependency.
On 2026-09-27 the assistant validated the run and CSV timing/target values and
inspected the plot. The learner completed sampling and plot interpretation exercises,
including correcting the angle comparison used to assess overshoot; see the lesson note.
