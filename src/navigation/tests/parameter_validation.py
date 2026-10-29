"""Check that invalid server parameters fail before the action is advertised."""

import subprocess
import resource

from test_support import installed_executable


# This test intentionally makes the C++ process fail at startup. Avoid a core file.
resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
result = subprocess.run(
    [
        installed_executable("turtle_navigation", "navigation_action_server"),
        "--ros-args", "-p", "controller_stop_timeout:=-1.0",
    ],
    capture_output=True, text=True, timeout=5.0, check=False,
)
assert result.returncode != 0
# Check the validation reason, not just a nonzero exit from an unrelated crash.
assert "Navigation action server parameters must be finite and in range" in result.stderr
print("PASS invalid parameter rejected at startup", flush=True)
