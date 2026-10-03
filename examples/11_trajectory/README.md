# Joint-space trajectory generation

These scripts generate UR5e joint-space references with NumPy. They do not run MuJoCo dynamics or
demonstrate actuator tracking.

## S10.8a — Linear

```bash
python examples/11_trajectory/linear.py
python examples/11_trajectory/linear.py --duration 4
```

The script saves a CSV and PNG under the ignored `tmp/11_linear_trajectory/` directory. The command
includes one stationary sample before and after the motion so the start/stop velocity jumps are visible.
Changing duration from 2 to 4 seconds should halve the constant in-motion velocity. The apparent endpoint
acceleration spikes are finite-difference representations of ideal instantaneous velocity changes; their
magnitude depends on sampling `dt` and is not a physically realizable acceleration command.

## S10.8b — Cubic comparison

```bash
python examples/11_trajectory/cubic.py
python examples/11_trajectory/cubic.py --duration 4
```

The cubic time law `s=3 tau^2-2 tau^3` satisfies zero velocity at both endpoints. The generated CSV and
three-panel PNG under `tmp/11_cubic_trajectory/` compare the first joint's linear and cubic position,
velocity, and acceleration. For the same displacement and duration, cubic motion has a 1.5-times larger
peak velocity than linear motion, but removes its endpoint velocity jumps. Cubic acceleration is finite
inside the motion but jumps where it meets the stationary holds, so the trajectory is C1, not C2.
