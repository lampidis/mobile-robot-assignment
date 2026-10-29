#!/usr/bin/env python3
"""Run each navigation integration scenario in a separate ROS domain."""

import argparse
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path


TEST_DIR = Path(__file__).resolve().parent
WORKSPACE = TEST_DIR.parents[2]


@dataclass(frozen=True)
class Case:
    name: str
    script: str
    args: tuple[str, ...] = ()
    fixture: str | None = None
    timeout: float = 35.0


CASES = (
    # The first three cases use a launch fixture. Fault cases start only the
    # processes they need so they can replace or kill the controller.
    Case("real navigation and validation", "real_navigation.py", fixture="turtlesim"),
    Case("cancel versus success race", "cancel_success_race.py", fixture="turtlesim"),
    Case("missing, invalid and stale pose", "pose_navigation.py", fixture="nodes"),
    Case("controller unavailable", "no_controller.py"),
    Case("controller crash during motion", "controller_crash.py"),
    Case("controller rejects request", "fault_navigation.py", ("reject",)),
    Case("controller sends no feedback", "fault_navigation.py", ("no_feedback",)),
    Case("client cancel stop timeout", "fault_navigation.py", ("stop_timeout",)),
    Case("preemption stop timeout", "fault_navigation.py", ("preempt_stop_timeout",)),
    Case("missing stop confirmation", "fault_navigation.py", ("bad_ack",)),
    Case("late controller acceptance", "fault_navigation.py", ("delayed_accept",)),
    Case("delayed stop before replacement", "fault_navigation.py", ("delayed_stop",)),
    Case("cancel pending goal", "fault_navigation.py", ("pending_cancel",)),
    Case("controller abort result", "fault_navigation.py", ("controller_abort",)),
    Case("invalid server parameters", "parameter_validation.py"),
)


def terminate_group(process: subprocess.Popen[str] | None) -> str:
    if process is None:
        return ""
    # A wrapper may have exited while a child in its process group still runs.
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    try:
        output, _ = process.communicate(timeout=3.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        output, _ = process.communicate(timeout=3.0)
    return output or ""


def run_case(case: Case, domain_id: int) -> tuple[bool, float, str, str]:
    # Each case gets its own ROS graph and a headless Qt backend for Turtlesim.
    environment = os.environ.copy()
    environment.update(
        ROS_DOMAIN_ID=str(domain_id),
        ROS_LOCALHOST_ONLY="1",
        QT_QPA_PLATFORM="offscreen",
        PYTHONUNBUFFERED="1",
    )
    fixture = None
    start = time.monotonic()
    output = ""
    fixture_output = ""
    passed = False
    try:
        if case.fixture:
            fixture = subprocess.Popen(
                [
                    "ros2", "launch", "turtle_navigation", "navigation.launch.py",
                    f"start_turtlesim:={'true' if case.fixture == 'turtlesim' else 'false'}",
                ],
                cwd=WORKSPACE, env=environment, start_new_session=True,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            )
        command = [sys.executable, str(TEST_DIR / case.script), *case.args]
        test = subprocess.Popen(
            command, cwd=WORKSPACE, env=environment, start_new_session=True,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        )
        try:
            output, _ = test.communicate(timeout=case.timeout)
            passed = test.returncode == 0
        except subprocess.TimeoutExpired:
            output = terminate_group(test) + f"\nTimed out after {case.timeout:g} seconds."
    except Exception as error:
        output += f"\nRunner error: {error}"
    finally:
        # Stop the fixture even if the scenario fails or exceeds its timeout.
        fixture_output = terminate_group(fixture)
    return passed, time.monotonic() - start, output, fixture_output


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", help="Run only cases whose names contain this text")
    parser.add_argument("--list", action="store_true", help="List all cases without running them")
    args = parser.parse_args()
    selected = [case for case in CASES if not args.case or args.case.lower() in case.name.lower()]
    if not selected:
        parser.error(f"no case matches {args.case!r}")
    if args.list:
        for case in selected:
            print(case.name)
        return 0

    print(f"Navigation integration tests: {len(selected)} cases", flush=True)
    results = []
    for index, case in enumerate(selected):
        # Different domains also prevent discovery state from one case affecting the next.
        passed, elapsed, output, fixture_output = run_case(case, 170 + index)
        label = "PASS" if passed else "FAIL"
        print(f"[{label}] {case.name} ({elapsed:.1f}s)", flush=True)
        if passed:
            for line in output.splitlines():
                if line.startswith("PASS "):
                    print(f"    {line}", flush=True)
        else:
            if output.strip():
                print(output.rstrip(), flush=True)
            if fixture_output.strip():
                print("Launch output:\n" + fixture_output.rstrip(), flush=True)
        results.append((case.name, passed, elapsed))

    passed_count = sum(passed for _, passed, _ in results)
    print("\nSummary", flush=True)
    for name, passed, elapsed in results:
        print(f"  {'PASS' if passed else 'FAIL':4}  {name:<38} {elapsed:5.1f}s", flush=True)
    print(f"{passed_count}/{len(results)} passed", flush=True)
    return 0 if passed_count == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
