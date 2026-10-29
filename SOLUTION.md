# Candidate solution — navigation

This document covers the navigation application as it stands today. The repository also contains the start of the separate perception application, but its full pipeline and launch file are not yet implemented. The [root assignment](README.md) requires both parts for a complete submission.

## Environment and dependencies

I developed and tested the navigation stack with **ROS 2 Humble on Ubuntu 22.04**, using the repository's [Dockerfile](Dockerfile). A Linux development machine with Docker is enough to follow the commands below; ROS does not need to be installed on the host. The image contains the C++ build tools, `colcon`, ROS 2 Humble, and Turtlesim. It also includes dependencies intended for the perception task.

The source lives in one ROS 2 workspace. Build from the repository root; do not create a second workspace under `src/navigation`.

## Build and run

From the repository root on the **host**, build the image and start a container. The bind mount keeps source changes and build outputs on the host. Running as your user avoids root-owned files.

```bash
docker build -t mobile-robot:humble .
docker run --rm -it --name mobile-robot-dev \
  --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -v /etc/passwd:/etc/passwd:ro -v /etc/group:/etc/group:ro \
  -v "$PWD:/workspace" -w /workspace \
  mobile-robot:humble bash
```

Inside the **container**, build the whole workspace, then launch Turtlesim and both navigation nodes. `QT_QPA_PLATFORM=offscreen` runs Turtlesim without a desktop display.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source /workspace/install/setup.bash
QT_QPA_PLATFORM=offscreen ros2 launch turtle_navigation navigation.launch.py start_turtlesim:=true
```

Leave that shell running. In another **host** terminal, open a shell in the same container and send a goal:

```bash
docker exec -it mobile-robot-dev bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
ros2 action send_goal /navigate_turtle turtle_nav_interfaces/action/NavigateTurtle "{x: 8.0, y: 5.5}" --feedback
```

The result should be `SUCCEEDED`, with distance feedback during movement and a final distance within `0.15` Turtlesim units. The launch file also accepts `start_turtlesim:=false` if Turtlesim is already running. To see the Turtlesim window, a Linux desktop needs X11 display forwarding into the container; the headless commands above need no display setup.

## How the navigation stack works

| Part | Responsibility |
| --- | --- |
| [`turtle_nav_interfaces`](src/navigation/turtle_nav_interfaces/) | Builds the public `NavigateTurtle` action and the internal `ControllerNavigate` action. Keeping message definitions in their own package lets both C++ nodes depend on the same generated types. |
| [`navigation_action_server`](src/navigation/turtle_navigation/src/navigation_action_server.cpp) | Offers `/navigate_turtle` to clients. It validates goals, tracks cancellation and replacement, forwards distance feedback, and decides each public result. |
| [`navigation_controller`](src/navigation/turtle_navigation/src/navigation_controller.cpp) | Reads `/turtle1/pose` and is the only node that publishes `/turtle1/cmd_vel`. It turns and drives toward the current target. |
| [`navigation.launch.py`](src/navigation/turtle_navigation/launch/navigation.launch.py) | Starts the public server, controller, and optionally Turtlesim. Both nodes read [`navigation.yaml`](src/navigation/turtle_navigation/config/navigation.yaml). |

The public server talks to the controller through `/controller/navigate`, a second ROS 2 action. It is an **internal convention**, not a ROS access-control boundary: other ROS participants can discover and call it. The internal result adds `stop_confirmed`, which means the controller published a zero velocity command before finishing. It does not prove that a physical robot has come to rest.

The controller uses a proportional turn and forward command at 20 Hz. It turns in place when its heading error is large, limits both speeds, and succeeds when the target is within the configured `0.15` unit position tolerance. I chose a small but forgiving tolerance for Turtlesim: exact coordinate equality would be unreliable at a fixed control rate. Final orientation is not part of this task. The gains, speed limits, tolerance, pose timeout, coordinate bounds, and server timeouts are in `navigation.yaml`.

## Goal and concurrency decisions

An accepted public goal is not immediately a moving turtle. The server first waits for the controller, sends it a separate goal asynchronously, and then forwards that goal's feedback. Goal and cancel callbacks return quickly; a 50 ms timer advances the server's goal state and checks deadlines. Each node runs on a single executor thread, so its own goal state changes in a defined callback order without extra mutexes.

The server keeps **one active goal** and **at most one pending replacement**. The controller also accepts only one goal at a time. This makes the latest-goal-wins policy concrete:

1. When B arrives while A is moving, the server asks the controller to stop A. B waits.
2. Once the controller confirms a zero command and returns A's result, A finishes `ABORTED` with a preemption message; B can then start.
3. If C arrives before B starts, B finishes `ABORTED` as superseded, and C becomes the pending goal. B never commands motion.

A client's cancel request follows a different path. The public server asks the controller to stop and normally reports `CANCELED` only after the controller returns its stop acknowledgment. A pending goal can be canceled without contacting the controller because it has never moved. For a cancel close to success, the first terminal decision handled by the server's executor wins; an accepted cancel cannot later become a public success. Dispatch tokens and ROS goal IDs keep late feedback or results from an older controller goal from finishing a newer public goal.

The server rejects non-finite or out-of-range targets (`x` and `y` must each be from `0.5` to `10.5`). It aborts a goal if the controller cannot start, rejects a request, stops sending feedback, or reports a failure. Start, feedback, and stop waits are bounded. If it cannot tell whether a controller request started motion, or cannot confirm a stop, it rejects further goals until the server and controller are restarted. For a client goal already in ROS `CANCELING`, the terminal status remains `CANCELED`, but its result message explicitly says when stopping was **not confirmed**. A result message must be read as well as the status.

## Reproduce the scenarios

From the repository root on the **host**, with the image built, run:

```bash
./src/navigation/tests/run_all.sh
```

The script builds the navigation packages and runs all tests in headless, isolated ROS domains. It prints a result for each case, then a summary, and exits nonzero if any case fails. It can also be run from a ROS Humble container. Use `--list` to see case names or `--case cancel` to select matching cases.

| Required scenario | Check in the suite |
| --- | --- |
| Normal navigation | Success, distance feedback, tolerance, and final zero velocity. |
| Cancellation | Active goal returns `CANCELED` after a stop; pending cancellation never reaches the controller. |
| Preemption | A is stopped and aborted before B can take the controller slot. |
| Rapid A/B/C requests | A aborts, B is superseded before dispatch, C succeeds. |
| Invalid goal | NaN, infinity, and coordinates outside the allowed area are rejected. |

The same suite also checks cancellation near success, missing or stale pose, a missing or crashed controller, delayed acceptance, missing feedback, stop timeouts, and bad stop acknowledgment. The test scripts are in [`src/navigation/tests`](src/navigation/tests/).

## Limits and unfinished work

The controller is the sole velocity publisher, as required. If that process dies abruptly after sending a nonzero command, the public server can detect the missing stop acknowledgment and block new goals, but it cannot send a zero velocity command itself. Immediate stopping after such a crash would need an independent velocity watchdog or another publisher, which would change this single-publisher design.

The perception application is still incomplete. It needs its camera calibration output, camera information and TF node, person detector, 3D location estimator, a complete launch file, and a repeatable demonstration using the supplied dataset before the repository meets the root assignment's full submission requirements.
