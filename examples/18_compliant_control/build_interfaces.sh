#!/usr/bin/env bash
set -euo pipefail
[[ -z "${CONDA_PREFIX:-}" && -z "${CONDA_DEFAULT_ENV:-}" && "${ROS_DISTRO:-}" == jazzy && "$(command -v python3)" == /usr/bin/python3 ]] || {
  echo 'Need ROS-only shell: empty conda, Jazzy, /usr/bin/python3' >&2; exit 1;
}
s17_repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$s17_repo"
pwd
echo "${CONDA_DEFAULT_ENV:-}"
which python3
cmake -S examples/18_compliant_control/s17_interfaces -B tmp/s17_9_interfaces/build \
  -DCMAKE_INSTALL_PREFIX="$s17_repo/tmp/s17_9_interfaces/install" \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=OFF
cmake --build tmp/s17_9_interfaces/build -j2
cmake --install tmp/s17_9_interfaces/build
