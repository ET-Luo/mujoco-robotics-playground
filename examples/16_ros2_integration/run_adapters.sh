#!/usr/bin/env bash
set -eo pipefail
s15_repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$s15_repo"
if [[ ! -f tmp/s15_6a_interfaces/install/share/s15_interfaces/local_setup.bash ]]; then
  echo 'Build interfaces in a ROS-only shell first; see S15.6a README' >&2; exit 1
fi
mkdir -p tmp/s15_6a
s15_out=$(mktemp -d "$s15_repo/tmp/s15_6a/run_XXXXXX")
s15_worker_pid= s15_server_pid= s15_perception_pid=
s15_cleanup() {
  for s15_pid in "$s15_perception_pid" "$s15_server_pid" "$s15_worker_pid"; do
    if [[ -n "$s15_pid" ]]; then kill -INT "$s15_pid" 2>/dev/null || true; fi
  done
  sleep .3
  for s15_pid in "$s15_perception_pid" "$s15_server_pid" "$s15_worker_pid"; do
    if [[ -n "$s15_pid" ]]; then kill -TERM "$s15_pid" 2>/dev/null || true; wait "$s15_pid" 2>/dev/null || true; fi
  done
}
trap s15_cleanup EXIT
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
[[ "$CONDA_DEFAULT_ENV" == mujoco && "$(command -v python)" == "$CONDA_PREFIX/bin/python" ]] || exit 1
python -u examples/16_ros2_integration/adapter_worker.py --socket "$s15_out/worker.sock" > "$s15_out/worker.log" 2>&1 &
s15_worker_pid=$!
for s15_i in $(seq 1 200); do
  [[ -S "$s15_out/worker.sock" ]] && break
  kill -0 "$s15_worker_pid" 2>/dev/null || { cat "$s15_out/worker.log"; exit 1; }
  sleep .1
done
[[ -S "$s15_out/worker.sock" ]] || { echo 'worker startup timeout'; exit 1; }
while [[ -n "${CONDA_PREFIX:-}" ]]; do conda deactivate; done
source /opt/ros/jazzy/setup.bash
source tmp/s15_6a_interfaces/install/share/s15_interfaces/local_setup.bash
export ROS_DOMAIN_ID=56
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
pwd
echo "${CONDA_DEFAULT_ENV:-}"
which python3
[[ -z "${CONDA_PREFIX:-}" && "$(command -v python3)" == /usr/bin/python3 && "$ROS_DISTRO" == jazzy ]] || exit 1
/usr/bin/python3 -u examples/16_ros2_integration/ros_adapters.py perception --socket "$s15_out/worker.sock" > "$s15_out/perception.log" 2>&1 &
s15_perception_pid=$!
/usr/bin/python3 -u examples/16_ros2_integration/ros_adapters.py server --socket "$s15_out/worker.sock" > "$s15_out/server.log" 2>&1 &
s15_server_pid=$!
echo "output=$s15_out"
set +e
/usr/bin/python3 -u examples/16_ros2_integration/ros_adapters.py client --socket "$s15_out/worker.sock" "$@" | tee "$s15_out/client.log"
s15_code=${PIPESTATUS[0]}
set -e
# Inspect worker after terminal/rejection; this does not advance simulation.
/usr/bin/python3 - "$s15_out/worker.sock" "$s15_out/final_state.json" <<'PY_S15'
import json,sys,time
from pathlib import Path
sys.path.insert(0,'examples/16_ros2_integration')
from ros_adapters import rpc
before=rpc(Path(sys.argv[1]),dict(op='state'))
time.sleep(.1)
after=rpc(Path(sys.argv[1]),dict(op='state'))
Path(sys.argv[2]).write_text(json.dumps(dict(before=before,after=after),indent=2)+'\n')
print(f'worker_running={after["running"]}; sim_time_s={after["sim_time_s"]}; state_stable={before==after}')
PY_S15
exit "$s15_code"
