#!/usr/bin/env bash
# Diagnose the caller's environment; never activate conda here.
set -euo pipefail

printf 'Working directory: %s\n' "$PWD"
printf 'Linux / WSL kernel: '
uname -sr
if [[ -r /etc/os-release ]]; then
    cat /etc/os-release
fi
printf 'Conda environment: %s\n' "${CONDA_DEFAULT_ENV:-<unset>}"
printf 'Python executable: %s\n' "$(command -v python || true)"

if [[ "${CONDA_DEFAULT_ENV:-}" != mujoco ]]; then
    echo 'FAIL: activate the required environment with: conda activate mujoco' >&2
    exit 1
fi
if [[ -z "${CONDA_PREFIX:-}" ]] || ! command -v python >/dev/null; then
    echo 'FAIL: conda prefix or Python executable is missing.' >&2
    exit 1
fi
if [[ "$(readlink -f "$(command -v python)")" != "$(readlink -f "$CONDA_PREFIX")/bin/"* ]]; then
    echo 'FAIL: Python is outside the active conda environment.' >&2
    exit 1
fi

python --version
if ! python - <<'PY'
import mujoco

print(f"MuJoCo version: {mujoco.__version__}")
PY
then
    echo 'FAIL: Python could not import MuJoCo; inspect the error above.' >&2
    exit 1
fi
echo 'PASS: the mujoco conda environment and MuJoCo import are ready.'
