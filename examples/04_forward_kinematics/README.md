# Forward kinematics

Status: documentation-only placeholder; no implementation yet.

Goal: Relate joint configuration to body and site poses.

S4.1 started on 2026-09-28: define world W, a rotating joint-local frame J, and
an end frame E using the Stage 3 single-link geometry. Work through p_W=R_WJ*p_J+t_WJ.
See [the lesson note](../../docs/05_forward_kinematics.md) for conventions and the +90-degree exercise.

The learner's manual answer is pending. No FK code, new model, or MuJoCo comparison
has been implemented or run. S4.2 two-link FK remains outside the current task.
