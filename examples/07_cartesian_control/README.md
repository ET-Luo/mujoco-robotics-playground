# Cartesian control

Status: documentation-only placeholder; no implementation yet.

Goal: Express a motion objective in end-effector coordinates.

TODO: Identify frame conventions and explain the mapping to joint commands first.

S6.1 has started as a concept exercise. The position target and error are expressed
in world coordinates and meters; the Jacobian maps a small world-position correction
to joint increments in radians. No controller, simulation, or GUI run exists yet.
