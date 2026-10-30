#!/usr/bin/env bash
set -eo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$workspace_root"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # Build current source before any test imports installed ROS packages.
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install --packages-up-to human_detection_node
  source "$workspace_root/install/setup.bash"
  exec python3 "$workspace_root/src/perception/tests/run_all.py" "$@"
fi

if ! command -v docker >/dev/null 2>&1; then
  printf 'ROS Humble and Docker are both unavailable.\n' >&2
  exit 2
fi

# All tests and the bag player share this isolated container's ROS graph.
exec docker run --rm --network none \
  --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -v /etc/passwd:/etc/passwd:ro -v /etc/group:/etc/group:ro \
  -v "$workspace_root:/workspace" -w /workspace \
  mobile-robot:humble bash src/perception/tests/run_all.sh "$@"
