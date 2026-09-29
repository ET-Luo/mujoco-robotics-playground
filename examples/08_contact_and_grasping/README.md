# Contact and grasping

Status: documentation-only placeholder; no implementation yet.

Goal: Observe how geometry, collision, and friction affect interaction.

TODO: Inspect contacts in a minimal scene before attempting a grasp.

S7.1 uses [contact_scene.xml](contact_scene.xml) and [main.py](main.py) to drop a
0.05 m radius free sphere from center height 0.20 m onto a plane at z=0. The learner
correctly predicted no initial contact, first contact near center height 0.05 m, and
unchanged model dimensions.

On 2026-09-29 the assistant ran the script in WSL2 Ubuntu 24.04.5 with the checked
mujoco environment, Python 3.12.14 and MuJoCo 3.13.0. It exited 0 without import errors.
Initial `ncon` was 0. First contact occurred at step 88, t=0.176 s, ball center
z=0.049789280 m, between `ground` and `ball_collision`. The world contact z was
-0.00010536 m and signed geom distance -0.00021072 m, showing slight penetration at
the discrete detection step. Model dimensions remained nq=7, nv=6, njnt=1.
Learner interpretation is pending; no GUI was run and no dependency was added.

The learner confirmed that contacts are temporary MjData constraints that do not change
model topology, and that discrete stepping plus soft contact explains the slight
penetration. S7.1 is complete. S7.2 has started with a two-slide-joint parallel-gripper
command and opening-width exercise.

[gripper.xml](gripper.xml) defines two opposite-axis slide joints and one position
actuator per finger; [gripper.py](gripper.py) verifies name/transmission mappings and
tests opening without an object. On 2026-09-29 it ran in WSL2 Ubuntu 24.04.5 in the
checked mujoco environment with Python 3.12.14/MuJoCo 3.13.0: exit 0, no import errors.
Initial qpos (0.01, 0.01) m gave 0.04 m opening. Writing ctrl (0.03, 0.03) m did not
immediately change qpos. After 1000 steps (2 s), qpos reached (0.03, 0.03) m and opening
was 0.08 m. Target and symmetry assertions passed. Learner interpretation is pending;
no object contact or GUI was tested.

The learner confirmed the two-finger opening change, actuator-to-joint target mapping,
and why unloaded opening does not prove grasp success. S7.2 is complete. S7.3 has
started with a world-frame Euclidean position-distance criterion using a strict 0.01 m
tolerance. [reach_criterion.py](reach_criterion.py) checks inside, exact-boundary, and
outside cases. Its first 2026-09-29 run exposed a test-only `numpy.bool_ is True`
identity error; after returning a Python bool and constructing a stable boundary case,
the rerun exited 0 under Python 3.12.14. Results were 0.006324555 m/True,
0.010000000 m/False, and 0.010001000 m/False. No motion controller or GUI was run.
Learner interpretation is pending.

The learner correctly distinguished position Reach from grasp success and classified a
0.012 m residual at timeout as unsuccessful. S7.3 is complete. S7.4 has started with
predictions for symmetric fingers closing from 0.08 m opening around a centered 0.04 m
object. [gripper_contact.xml](gripper_contact.xml) places a free box in the zero-gravity
gripper; [close_contact.py](close_contact.py) checks geom-name sets without assuming
contact order. On 2026-09-29 it ran in the checked mujoco environment with Python
3.12.14/MuJoCo 3.13.0: exit 0, no import errors. Simultaneous left/object and
right/object contact appeared at step 27, t=0.054 s. Finger qpos was approximately
(0.00901662, 0.00901662) m against ctrl (0.005, 0.005) m; opening was 0.038033237 m
around the 0.04 m object due to soft-contact penetration. The centered object had not
moved. This verifies a two-sided contact event, not stable grasping or lifting.

The learner confirmed that contact reaction prevents qpos from reaching ctrl and that
the script returns on first simultaneous contact rather than a joint-position tolerance.
S7.4 is complete. S7.5 has started by defining a small-lift observation using object
world-height change and object-to-gripper relative vertical change; no lift scene exists yet.

[gripper_lift.xml](gripper_lift.xml) and [lift.py](lift.py) test a pregrasped object under
gravity using a ramped 0.05 m base target. Two intermediate runs correctly failed the
unchanged criterion: kp=500/step command lifted the object only 0.016736 m; kp=2000/ramp
lifted it 0.036966 m. With lift kp=5000, the 2026-09-29 run exited 0 under Python
3.12.14/MuJoCo 3.13.0. The base rose 0.047886057 m, the object rose 0.040620966 m,
and relative z changed -0.007265091 m, satisfying the >=0.04 m lift and <=0.01 m slip
criteria. No GUI was run. Learner interpretation is pending.

The learner confirmed finite-stiffness gravitational sag and the downward meaning of
negative relative z. S7.5 is complete. S7.6 has started with the sequence and evidence
for supported placement, finger release, and gripper retreat; no place/release run yet.

[place_release.py](place_release.py) reproduces the held/lifted state, descends until an
object-ground contact is detected, opens the fingers, retreats, and checks support,
finger separation, xy placement, and object speed. Extending the lift joint range was
regression-checked with `lift.py`, which still passed (object lift 0.042733988 m,
relative change -0.007265089 m). The 2026-09-29 place run exited 0 under Python
3.12.14/MuJoCo 3.13.0. Support appeared at object z=0.029914602 m. Final object position
was approximately (0, 0, 0.029362824) m, speed 1.36e-10 m/s, ground support True,
finger contact False, and `Place success=True`. No GUI was run; learner interpretation
and the combined-task checklist are pending.

The learner confirmed how each final check rules out a distinct placement failure.
S7.6 and Stage 7 are complete. The full Reach→align→close→hold→lift→transfer→descend→
support→release→retreat→final-check sequence is recorded in the manipulation lesson note;
it is a checklist, not an implemented pick-and-place state machine.
