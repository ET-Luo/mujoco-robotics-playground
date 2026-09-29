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
command and opening-width exercise; no gripper model exists yet.
