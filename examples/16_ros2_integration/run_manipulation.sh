#!/usr/bin/env bash
set -eo pipefail
s15_repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$s15_repo"
[[ -f tmp/s15_6a_interfaces/install/share/s15_interfaces/local_setup.bash ]] || { echo 'Build interfaces first'; exit 1; }
mkdir -p tmp/s15_6b
s15_out=$(mktemp -d "$s15_repo/tmp/s15_6b/run_XXXXXX")
s15_worker_pid= s15_server_pid= s15_perception_pid= s15_rsp_pid=
s15_cleanup() {
  for s15_pid in "$s15_perception_pid" "$s15_server_pid" "$s15_rsp_pid" "$s15_worker_pid"; do
    [[ -z "$s15_pid" ]] || kill -INT "$s15_pid" 2>/dev/null || true
  done
  sleep .3
  for s15_pid in "$s15_perception_pid" "$s15_server_pid" "$s15_rsp_pid" "$s15_worker_pid"; do
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
python -u examples/16_ros2_integration/manipulation_worker.py --socket "$s15_out/worker.sock" --out "$s15_out/artifacts" --close-target "${S15_CLOSE_TARGET:-0.005}" > "$s15_out/worker.log" 2>&1 &
s15_worker_pid=$!
for s15_i in $(seq 1 300); do
  [[ -S "$s15_out/worker.sock" ]] && break
  kill -0 "$s15_worker_pid" 2>/dev/null || { cat "$s15_out/worker.log"; exit 1; }
  sleep .1
done
[[ -S "$s15_out/worker.sock" ]] || { echo 'worker startup timeout'; exit 1; }
while [[ -n "${CONDA_PREFIX:-}" ]]; do conda deactivate; done
source /opt/ros/jazzy/setup.bash
source tmp/s15_6a_interfaces/install/share/s15_interfaces/local_setup.bash
export ROS_DOMAIN_ID=57
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
pwd
echo "${CONDA_DEFAULT_ENV:-}"
which python3
printenv ROS_DISTRO
[[ -z "${CONDA_PREFIX:-}" && "$(command -v python3)" == /usr/bin/python3 && "$ROS_DISTRO" == jazzy ]] || exit 1
s15_rsp=$(/usr/bin/python3 -c 'from ament_index_python.packages import get_package_prefix; print(get_package_prefix("robot_state_publisher")+"/lib/robot_state_publisher/robot_state_publisher")')
"$s15_rsp" --ros-args -p use_sim_time:=true -p publish_frequency:=1000.0 -p "robot_description:=$(cat examples/16_ros2_integration/ur5e_kinematics.urdf)" > "$s15_out/rsp.log" 2>&1 &
s15_rsp_pid=$!
/usr/bin/python3 -u examples/16_ros2_integration/manipulation_ros.py perception --socket "$s15_out/worker.sock" > "$s15_out/perception.log" 2>&1 &
s15_perception_pid=$!
/usr/bin/python3 -u examples/16_ros2_integration/manipulation_ros.py server --socket "$s15_out/worker.sock" > "$s15_out/server.log" 2>&1 &
s15_server_pid=$!
echo "output=$s15_out"
set +e
/usr/bin/python3 -u examples/16_ros2_integration/manipulation_ros.py client --socket "$s15_out/worker.sock" --output "$s15_out/client_result.json" "$@" | tee "$s15_out/client.log"
s15_code=${PIPESTATUS[0]}
set -e
/usr/bin/python3 - "$s15_out/worker.sock" "$s15_out/final_state.json" <<'PY_S15'
import json,sys,time
from pathlib import Path
sys.path.insert(0,'examples/16_ros2_integration')
from manipulation_ros import rpc
before=rpc(Path(sys.argv[1]),dict(op='state'))
time.sleep(.15)
after=rpc(Path(sys.argv[1]),dict(op='state'))
Path(sys.argv[2]).write_text(json.dumps(dict(before=before,after=after),indent=2)+'\n')
print(f'worker_running={after["running"]}; state_stable={before==after}; sim_time_s={after["state"]["simulation_time_s"]}')
PY_S15
exit "$s15_code"
