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
tolerance; no Reach criterion script or motion controller exists yet.
