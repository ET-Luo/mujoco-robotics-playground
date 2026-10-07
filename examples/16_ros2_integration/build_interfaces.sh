#!/usr/bin/env bash
set -euo pipefail
# ROS-only build shell. Caller must deactivate conda and source Jazzy first.
if [[ -n "${CONDA_PREFIX:-}" || "${ROS_DISTRO:-}" != jazzy || "$(command -v python3)" != /usr/bin/python3 ]]; then
  echo 'Need empty conda, Jazzy setup, /usr/bin/python3' >&2; exit 1
fi
pwd
echo "${CONDA_DEFAULT_ENV:-}"
which python3
s15_repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
cmake -S "$s15_repo/examples/16_ros2_integration/s15_interfaces" -B "$s15_repo/tmp/s15_6a_interfaces/build" \
  -DCMAKE_INSTALL_PREFIX="$s15_repo/tmp/s15_6a_interfaces/install" \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=OFF
cmake --build "$s15_repo/tmp/s15_6a_interfaces/build" -j2
cmake --install "$s15_repo/tmp/s15_6a_interfaces/build"
