# Candidate solution — navigation and perception calibration

This document covers the validated navigation application and the perception environment and camera calibration (Parts 0 and 1). Person detection, 3D location, and the complete perception demonstration remain later steps. Both tasks use the same root Docker image and ROS 2 workspace.

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

Perception calibration is documented below. The CameraInfo/TF publisher, person detector, 3D location estimator, complete launch, and full demonstration remain later work.

## Candidate solution — perception Parts 0 and 1

### Shared Docker environment

The root [Dockerfile](Dockerfile) builds one `mobile-robot:humble` image containing ROS 2 Humble, OpenCV, Python, and the dependencies for both tasks. The raw calibration photos and ROS bag are not copied into that image or Git. Run one container with the repository mounted at `/workspace`; this mount also preserves generated calibration files and results on the host. From the repository root on the host:

```bash
docker build -t mobile-robot:humble .
docker run --rm -it --name mobile-robot-calibration --network host \
  --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -v "$PWD:/workspace" \
  -v "$PWD/src/perception/data:/workspace/src/perception/data:ro" \
  -w /workspace mobile-robot:humble bash
```

Put the supplied 32 PNG photos under `src/perception/data/calibration_images/` and the bag's `metadata.yaml` and `person_walking_0.db3` under `src/perception/data/rosbag/person_walking/` before starting. In the container:

```bash
source /opt/ros/humble/setup.bash
python3 src/perception/scripts/camera_calibration.py
colcon build --symlink-install
ros2 bag info src/perception/data/rosbag/person_walking
```

The `colcon` command builds from the repository root. `ros2 bag info` inspects the supplied recording; when testing playback, use `ros2 bag play src/perception/data/rosbag/person_walking` in another shell. Bag playback is separate from calibration. Outputs remain on the host through the workspace mount.

### Fisheye calibration method and reasons

The [calibration script](src/perception/scripts/camera_calibration.py) finds a 9×7 grid of inner chessboard corners. Adjacent corners are 20 mm apart, represented as 0.020 metres on a flat board. It pairs these known 3D positions with the detected 2D pixel positions in each photo. Unreadable photos, inconsistent image sizes, and photos without all 63 corners are reported as unusable. The supplied photos and recorded camera images are 1280×720.

The script uses OpenCV's `cv2.fisheye.calibrate` because the supplied camera has a wide-angle lens and the assignment requires a fisheye model. It excludes `camera0_20261007162543.png`: visual review found that the board is warped, whereas the calibration model assumes a flat board. The accepted and rejected filenames are saved in the [report](src/perception/results/calibration_report.json). The remaining 31 photos enter **one** fisheye fit.

A nonlinear fit needs starting camera parameters. The actual lens focal length and field of view were not supplied. For an approximately equidistant lens, image radius is focal length times viewing angle. Taking half the 1280-pixel width as an approximate 90° radius gives `f₀ = (1280/2)/(π/2) = 1280/π ≈ 407.44` pixels. The initial principal point is the image centre `(640, 360)` and the four initial distortion coefficients are zero. OpenCV then optimizes these parameters using `CALIB_USE_INTRINSIC_GUESS | CALIB_RECOMPUTE_EXTRINSIC`; the assumed 180° field of view is only an initialization heuristic. See [OpenCV's fisheye documentation](https://docs.opencv.org/4.10.0/db/d58/group__calib3d__fisheye.html) and [Kannala and Brandt's fisheye calibration work](https://users.aalto.fi/~kannalj1/publications/tpami2006.pdf).

The initialization was tested on the 31 selected images. OpenCV's automatic guess failed in `InitExtrinsics` in three direct runs, as did a planar pinhole seed. The equidistant image-size seed succeeded in three runs with identical YAML output. This is a tested dataset-specific starting method, not a measured camera specification. The final overall RMS reprojection error is **0.654244 pixels**. The fitted focal lengths are about 481.09 and 476.63 pixels, and the fitted principal point is about `(587.78, 324.10)`. The result is saved as an `equidistant` [CameraInfo YAML](src/perception/front_camera_info/config/camera_calibration.yaml) with image dimensions, K, four D coefficients, identity R, and P.

The report lists per-image errors. Three visual examples show detected corners in [early](src/perception/results/calibration_corners.png), [middle](src/perception/results/calibration_corners_middle.png), and [late](src/perception/results/calibration_corners_last.png) accepted photos. Review that the coloured markers sit on real corner intersections. RMS measures fit to these calibration photos; it does not by itself prove later 3D person-location accuracy.
