#!/usr/bin/env bash
# Simple launcher that activates project venv, sources workspace, then runs a command.
# Usage:
#   ./run_with_venv.sh -- test-import
#   ./run_with_venv.sh -- ros2 run nyurdt_autonomy excavation

#Gad26. 'Added numpy and scipy to venv install instructions' AND ' added install_requires to setup.py files'
#Gad26. 'this is legit just a venv activator script and helper to kick things off on my own machine.'
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="$HERE/.venv"
WORKSPACE="$HERE"

if [ ! -d "$VENV" ]; then
  echo "Virtualenv not found at $VENV. Create it with: python3 -m venv $VENV && $VENV/bin/python3 -m pip install --upgrade pip setuptools wheel && $VENV/bin/pip install apriltag opencv-python-headless numpy"
  exit 1
fi

# Activate venv
# shellcheck source=/dev/null
. "$VENV/bin/activate"

# Source the ROS workspace install/setup.bash if present
if [ -f "$WORKSPACE/install/setup.bash" ]; then
  # Some setup scripts reference variables that may be unset; temporarily disable nounset
  set +u
  # shellcheck source=/dev/null
  source "$WORKSPACE/install/setup.bash" || true
  set -u
fi

# If no args or the first arg is -- test-import, run a quick apriltag import test
if [ "$#" -eq 0 ]; then
  python3 -c "import apriltag; print('apriltag OK ->', apriltag.__file__)"
  exit 0
fi

if [ "$1" = "--" ]; then
  shift
fi

# Run remaining arguments as a command
exec "$@"
