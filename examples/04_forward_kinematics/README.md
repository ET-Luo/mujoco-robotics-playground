# Forward kinematics

Status: S4.1 and S4.2 learning exercises complete; minimal NumPy FK implemented.

Goal: Relate joint configuration to body and site poses.

S4.1 started on 2026-09-28: define world W, a rotating joint-local frame J, and
an end frame E using the Stage 3 single-link geometry. Work through p_W=R_WJ*p_J+t_WJ.
See [the lesson note](../../docs/05_forward_kinematics.md) for conventions and the +90-degree exercise.

S4.1 is complete: the learner explained local versus world motion and correctly
computed the midpoint's rotated displacement (0, 0.2, 0) m and world position
(0, 0.2, 0.5) m. The assistant supplied the frame diagram and rotation matrix.
No new MuJoCo model or MuJoCo comparison has been implemented or run.
S4.2 geometry work has started: L1=0.4 m, L2=0.3 m, base at the world origin.
The learner correctly predicted the (q1,q2)=(0°,0°) and (90°,0°) endpoints.
The learner also correctly computed the (90°,90°) endpoint (-0.3,0.4,0) m
and supplied both FK formulas and the two core Python expressions.
The assistant integrated them into [main.py](main.py), with radian angle inputs,
meter link lengths, and a (3,) world-position output in meters.

After activating and verifying mujoco, run from the repository root:

```bash
python examples/04_forward_kinematics/main.py
```

On 2026-09-28, the assistant ran this in WSL2 Ubuntu 24.04.5, conda mujoco,
Python 3.12.14. Exit 0; the (0,0) and (π/2,π/2) outputs matched the learner's
hand calculations within 1e-12 m absolute tolerance. No physics or GUI was run.
The learner wrote the core formulas; the assistant supplied the wrapper and checks.

S4.3 predictions are correct. Run the [site pose example](site_pose.py) after
activating and verifying mujoco:

```bash
python examples/04_forward_kinematics/site_pose.py
```

It reads attachment_site at home, adds 0.1 rad to shoulder_pan qpos, and calls
mj_forward to refresh the world pose without stepping. Matrix columns are the
site axes in world coordinates. On 2026-09-28, the assistant ran it in WSL2
Ubuntu 24.04.5, conda mujoco, Python 3.12.14, MuJoCo 3.14.0: exit 0.
Local position and time stayed unchanged; world position and orientation changed.
No GUI was tested. The learner correctly identified the home site's +y/+z axes
as approximately world -y/-z. S4.3 is complete; see the lesson note for numeric results.
S4.4 position Jacobian work has not started.
