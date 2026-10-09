#!/usr/bin/env bash
set -eo pipefail
s17_repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$s17_repo"
source "$(conda info --base)/etc/profile.d/conda.sh"
# Build in isolated ROS environment without changing caller's conda worker environment.
(
  while [[ -n "${CONDA_PREFIX:-}" ]]; do conda deactivate; done
  unset PYTHONPATH
  source /opt/ros/jazzy/setup.bash
  bash examples/18_compliant_control/build_interfaces.sh > /tmp/s17_9_build.log 2>&1
) || { cat /tmp/s17_9_build.log; exit 1; }
mkdir -p tmp/s17_9
s17_out=$(mktemp -d "$s17_repo/tmp/s17_9/run_XXXXXX")
s17_worker_pid= s17_server_pid=
s17_cleanup() {
  if [[ -n "$s17_server_pid" ]]; then kill -INT "$s17_server_pid" 2>/dev/null || true; fi
  if [[ -n "$s17_worker_pid" ]]; then kill -INT "$s17_worker_pid" 2>/dev/null || true; fi
  sleep .5
  for s17_pid in "$s17_server_pid" "$s17_worker_pid"; do
    if [[ -n "$s17_pid" ]]; then kill -TERM "$s17_pid" 2>/dev/null || true; wait "$s17_pid" 2>/dev/null || true; fi
  done
}
trap s17_cleanup EXIT
conda activate mujoco
pwd
echo "$CONDA_DEFAULT_ENV"
which python
[[ "$CONDA_DEFAULT_ENV" == mujoco && "$(command -v python)" == "$CONDA_PREFIX/bin/python" ]] || exit 1
(unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH ROS_DISTRO; exec python -u examples/18_compliant_control/control_worker.py --socket "$s17_out/worker.sock" --output "$s17_out" > "$s17_out/worker.log" 2>&1) &
s17_worker_pid=$!
for s17_i in $(seq 1 200); do
  [[ -S "$s17_out/worker.sock" ]] && break
  kill -0 "$s17_worker_pid" 2>/dev/null || { cat "$s17_out/worker.log"; exit 1; }
  sleep .1
done
[[ -S "$s17_out/worker.sock" ]] || { echo 'worker startup timeout'; exit 1; }
while [[ -n "${CONDA_PREFIX:-}" ]]; do conda deactivate; done
  unset PYTHONPATH
source /opt/ros/jazzy/setup.bash
source tmp/s17_9_interfaces/install/share/s17_interfaces/local_setup.bash
export ROS_DOMAIN_ID=79
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
pwd
echo "${CONDA_DEFAULT_ENV:-} ${CONDA_PREFIX:-}"
which python3
echo "$ROS_DISTRO"
[[ -z "${CONDA_DEFAULT_ENV:-}" && -z "${CONDA_PREFIX:-}" && "$(command -v python3)" == /usr/bin/python3 && "$ROS_DISTRO" == jazzy ]] || exit 1
/usr/bin/python3 -u examples/18_compliant_control/worker_ros.py server --socket "$s17_out/worker.sock" --output "$s17_out" "$@" > "$s17_out/server.log" 2>&1 &
s17_server_pid=$!
echo "output=$s17_out"
/usr/bin/python3 -u examples/18_compliant_control/worker_ros.py client --socket "$s17_out/worker.sock" --output "$s17_out" "$@" | tee "$s17_out/client.log"
