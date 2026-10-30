#!/usr/bin/env python3
"""Run focused perception checks and print one navigation-style summary."""

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
TESTS = Path(__file__).resolve().parent
BAG = Path('src/perception/data/rosbag/person_walking')

# Each subprocess checks one behavior. The last case checks that the nodes
# communicate together with real recorded sensor messages.
CASES = (
    ('recorded bag contract', ('audit_bag.py', str(BAG)), 45),
    ('CameraInfo topic and bad image size', ('test_ros_topics.py', 'camera'), 20),
    ('detector topics and bad encoding', ('test_ros_topics.py', 'detector'), 40),
    ('locator missing cloud and empty frame', ('test_ros_topics.py', 'locator'), 20),
    ('3D selection edge cases', ('test_locator_selection.py',), 20),
    ('full pipeline communication', ('pipeline_smoke.py', '--duration', '22',
                                     '--rate', '0.5', '--start-offset', '15'), 65),
)


def main():
    """Run all cases, continue after failures, and return failure if any fail."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', help='run only cases whose names contain this text')
    parser.add_argument('--list', action='store_true', help='list the case names')
    args = parser.parse_args()
    selected = [case for case in CASES if not args.case or args.case.lower() in case[0].lower()]
    if not selected:
        parser.error(f'no case matches {args.case!r}')
    if args.list:
        for name, _, _ in selected:
            print(name)
        return 0

    print(f'Perception tests: {len(selected)} {"case" if len(selected) == 1 else "cases"}', flush=True)
    results = []
    for index, (name, command, timeout) in enumerate(selected):
        # Keep each ROS graph separate from other cases and manual RViz runs.
        domain = 180 + index
        environment = os.environ.copy()
        environment.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1',
                           PYTHONUNBUFFERED='1')
        argv = [sys.executable, str(TESTS / command[0]), *command[1:]]
        if command[0] == 'pipeline_smoke.py':
            argv += ['--domain', str(domain)]
        start = time.monotonic()
        process = subprocess.Popen(argv, cwd=ROOT, env=environment,
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                   text=True, start_new_session=True)
        try:
            output, _ = process.communicate(timeout=timeout)
            passed = process.returncode == 0
        except subprocess.TimeoutExpired:
            # The pipeline case starts launch and bag children. Stop its whole
            # process group so a timed-out test cannot affect later cases.
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                output, _ = process.communicate(timeout=4)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                output, _ = process.communicate()
            output += f'\nTimed out after {timeout}s.'
            passed = False
        elapsed = time.monotonic() - start
        results.append((name, passed, elapsed))
        print(f'[{"PASS" if passed else "FAIL"}] {name} ({elapsed:.1f}s)', flush=True)
        if not passed:
            print(output[-4000:].strip(), flush=True)
        else:
            for line in output.splitlines():
                if line.startswith('PASS '):
                    print(f'    {line}', flush=True)

    print('\nSummary', flush=True)
    for name, passed, elapsed in results:
        print(f'  {"PASS" if passed else "FAIL":4}  {name:<40} {elapsed:5.1f}s', flush=True)
    print(f'{sum(passed for _, passed, _ in results)}/{len(results)} passed', flush=True)
    return 0 if all(passed for _, passed, _ in results) else 1


if __name__ == '__main__':
    sys.exit(main())
