#!/usr/bin/env bash
set -eo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$workspace_root"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # Inside the image, rebuild the current workspace before testing its nodes.
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install --packages-up-to turtle_navigation
  source "$workspace_root/install/setup.bash"
  exec python3 "$workspace_root/src/navigation/tests/run_all.py" "$@"
fi

if ! command -v docker >/dev/null 2>&1; then
  printf 'ROS Humble and Docker are both unavailable.\n' >&2
  exit 2
fi

# A separate network namespace keeps the tests away from any running GUI session.
exec docker run --rm --network none \
  --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -v /etc/passwd:/etc/passwd:ro -v /etc/group:/etc/group:ro \
  -v "$workspace_root:/workspace" -w /workspace \
  mobile-robot:humble bash src/navigation/tests/run_all.sh "$@"
