# Joint-space trajectory generation

S10.8a studies a linear UR5e joint-space trajectory. It is a NumPy command generator, not a MuJoCo
dynamics or actuator-tracking experiment.

```bash
python examples/11_trajectory/linear.py
python examples/11_trajectory/linear.py --duration 4
```

The script saves a CSV and PNG under the ignored `tmp/11_linear_trajectory/` directory. The command
includes one stationary sample before and after the motion so the start/stop velocity jumps are visible.
Changing duration from 2 to 4 seconds should halve the constant in-motion velocity. The apparent endpoint
acceleration spikes are finite-difference representations of ideal instantaneous velocity changes; their
magnitude depends on sampling `dt` and is not a physically realizable acceleration command.

