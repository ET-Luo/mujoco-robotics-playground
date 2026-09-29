# Inverse kinematics

Status: documentation-only placeholder; no implementation yet.

Goal: Find a joint configuration for a desired end-effector pose.

TODO: Explain Jacobian updates, singularities, and stopping criteria before implementation.

S5.1 is complete with a planar two-link position-only hand calculation:
world-frame error, solving J*delta_q=error, step fraction alpha, and position
tolerance versus iteration-limit termination. See [the lesson note](../../docs/07_inverse_kinematics.md).
After correction of the error sign and half-step arithmetic, the learner correctly
predicted x=0.3985 m. An assistant FK calculation gives about 1.5 mm residual error;
the learner correctly explained that this exceeds the 0.1 mm success tolerance. No IK solver, controller,
or iteration loop is implemented.
