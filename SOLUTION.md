# Candidate solution: navigation and perception

This solution contains two independently launchable ROS 2 Humble applications in one workspace and Docker image: Turtlesim navigation and person detection with approximate 3D location from camera and LiDAR data. The commands below build, run, and test both tasks, the later sections explain the design choices and limits.

## 1. Prepare the shared workspace

I built and tested both applications with **ROS 2 Humble on Ubuntu 22.04**. The root [Dockerfile](Dockerfile) installs the dependencies for both tasks and downloads the pretrained `yolo26n.pt` model into `/opt/models/yolo26n.pt` with a checksum check. I chose a small model for CPU inference and pinned Ultralytics 8.4.190 because an older version missed person boxes with these weights. The host does not need ROS or Python packages installed.

Place the calibration PNGs under `src/perception/data/calibration_images/`. The recording needs `src/perception/data/rosbag/person_walking/metadata.yaml` and `person_walking_0.db3`. The raw data and model weights are not committed. The dataset mount is read only. Rerunning calibration updates the submitted `src/perception/front_camera_info/config/camera_calibration.yaml` and the report and corner images in `src/perception/results/`. Navigation test output goes under `out/navigation/`; projection reviews, perception test output, and optional output recordings go under `out/perception/`. Both output directories are ignored. The workspace mount keeps these files and the ignored `build/`, `install/`, and `log/` directories on the host. One container runs either task, so there is no Compose file.

From the repository root on a **Linux X11 desktop**, build the image and start the container. The host needs an active `DISPLAY` and an Xauthority file at `${XAUTHORITY:-$HOME/.Xauthority}`. Running as your host user keeps generated files owned by you.

```bash
docker build -t mobile-robot:humble .
docker run --rm -it --name mobile-robot-demo --network host \
  --user "$(id -u):$(id -g)" -e HOME=/tmp -e ROS_DOMAIN_ID=90 \
  -e DISPLAY="$DISPLAY" -e XAUTHORITY=/tmp/host.Xauthority \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v /etc/passwd:/etc/passwd:ro -v /etc/group:/etc/group:ro \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v "${XAUTHORITY:-$HOME/.Xauthority}:/tmp/host.Xauthority:ro" \
  -v "$PWD:/workspace" \
  -v "$PWD/src/perception/data:/workspace/src/perception/data:ro" \
  -w /workspace mobile-robot:humble
```

Inside the container, build the workspace once and load its packages. The image already loads `/opt/ros/humble/setup.bash` when it starts. The build installs the launch files, calibration YAML, sensor URDF, and RViz preset.

```bash
colcon build --symlink-install
source install/setup.bash
```

You can also work without a desktop. Start the image without the display options, then run the same build commands. Set `QT_QPA_PLATFORM=offscreen` for Turtlesim and pass `rviz:=false` to the perception demo. Display forwarding is only needed for the Turtlesim and RViz windows.

## 2. Navigation: run and verify

### Launch a goal

In the built container, launch the application and one example goal:

```bash
ros2 launch turtle_navigation navigation.launch.py demo_goal:=true
```

This starts Turtlesim, the public action server, and the controller. It then sends `(8.0, 5.5)` through `/navigate_turtle`. You should see the turtle move. The terminal shows distance feedback and should finish with `SUCCEEDED` and a final distance below `0.15` Turtlesim units. Press **Ctrl+C** to stop. The launch normally leaves `demo_goal` off so you can send your own goals.

### Send a goal manually

To choose your own goal, start `ros2 launch turtle_navigation navigation.launch.py` in the container, then open a second **host** terminal:

```bash
docker exec -it mobile-robot-demo bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
ros2 action send_goal /navigate_turtle turtle_nav_interfaces/action/NavigateTurtle "{x: 8.0, y: 5.5}" --feedback
```

The launch also accepts `start_turtlesim:=false` when another Turtlesim instance is already running.

### Reproduce the navigation scenarios

From the repository root on the **host**, with the image built, run the whole isolated navigation suite:

```bash
mkdir -p out/navigation
set -o pipefail
./src/navigation/tests/run_all.sh 2>&1 | tee out/navigation/tests.log
```

The runner builds the navigation packages, uses headless isolated ROS domains, prints every case and a summary, and exits nonzero on failure. The full test output is saved in `out/navigation/tests.log`; `pipefail` preserves the runner's failure status through `tee`. Use `./src/navigation/tests/run_all.sh --list` for case names or `--case cancel` for matching cases.

| Scenario | What the suite checks |
| --- | --- |
| Normal navigation | Success, distance feedback, position tolerance, and final zero velocity. |
| Cancellation | An active goal stops before `CANCELED`. Canceling a pending goal never reaches the controller. |
| Preemption and rapid A/B/C requests | A stops and aborts before replacement. B can be superseded before dispatch. C succeeds. |
| Invalid goals | NaN, infinity, and coordinates outside the allowed area are rejected. |
| Failure and race edges | Cancellation near success, missing or stale pose, missing or crashed controller, delayed acceptance, missing feedback, stop timeouts, bad stop acknowledgment, and late controller messages. |

## 3. Perception: run and verify

The [perception requirements](src/perception/README.md) cover calibration, ROS integration, person detection, and 3D location. The recorded publishers use best effort QoS, so a subscriber may receive fewer messages than the bag contains.

### Launch the complete demo

In the built container, start the complete visualization and input replay with one command:

```bash
ros2 launch human_detection_node perception_demo.launch.py
```

The [demo launch](src/perception/human_detection_node/launch/perception_demo.launch.py) starts CameraInfo, `robot_state_publisher` for TF, YOLO detection, 3D location, RViz, and looping playback. RViz opens with saved views for the images, raw LiDAR, TF, and selected person points in magenta. Press **Ctrl+C** to stop. On the X11 host used for testing, RViz worked during playback but reported exit code `-11` when the whole launch stopped.

The default playback rate is `0.5×` (half speed). I chose it to give CPU inference and LiDAR projection more time per recorded frame and make the RViz output easier to inspect. This rate is a demonstration setting, not a measured performance limit. Use `rate:=1.0` to try normal speed.

### Inspect and record playback

To run the nodes without automatic playback or RViz, use `ros2 launch human_detection_node perception.launch.py` instead. To inspect the full bag separately, open another **host** terminal and enter the same container:

```bash
docker exec -it mobile-robot-demo bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
ros2 bag info src/perception/data/rosbag/person_walking
ros2 bag play src/perception/data/rosbag/person_walking --rate 0.5
```

Keep the core launch running in the first shell while the second shell plays the recording. To inspect one location in another sourced shell, use `ros2 topic echo --once /human/location --qos-reliability best_effort`.

To save a full output recording, start the following in a third sourced shell **before** playback. Stop recording with Ctrl+C, then inspect the saved bag. The workspace mount keeps it on the host.

```bash
mkdir -p out/perception
ros2 bag record -o out/perception/full-run \
  /m20/camera/front/camera_info \
  /m20/camera/front/human/image_annotated \
  /m20/camera/front/human/detections /human/location /human/selected_points
ros2 bag info out/perception/full-run
```

While the core launch is running, you can also inspect the topic connections and the supplied sensor transform in a sourced shell:

```bash
ros2 topic info -v /m20/camera/front/camera_info
ros2 topic info -v /m20/camera/front/human/detections
ros2 topic info -v /human/location
ros2 run tf2_ros tf2_echo lidar_link front_wide_angle_camera_frame
```

The `tf2_echo` command keeps printing until you stop it with Ctrl+C. If you only need CameraInfo and TF, use `ros2 launch front_camera_info camera_info_tf.launch.py`.

### Regenerate calibration and projection reviews

To rerun the camera fit or inspect projection coverage, run these commands from `/workspace` in a sourced container shell. The calibration script regenerates the YAML and review images. It does not need bag playback.

```bash
python3 src/perception/scripts/camera_calibration.py
python3 src/perception/scripts/review_projection.py --frames 120 270
python3 src/perception/scripts/review_projection.py --coverage-all
```

The projection commands save overlays under `out/perception/projection-review/`. Blue and red show nearer and farther projected LiDAR points. In these overlays, magenta marks LiDAR points inside a detected person box that pass the floor filter, they are candidates, not necessarily the final selected group. A yellow cross marks the estimated location. In RViz, magenta instead shows the final selected points on `/human/selected_points`. The full coverage command saves `coverage-audit.json` and `coverage-rows.png` for all 371 clouds.

### Reproduce the perception tests

From the repository root on the **host**, with the image built and the supplied bag present, run every isolated perception case:

```bash
mkdir -p out/perception
set -o pipefail
./src/perception/tests/run_all.sh 2>&1 | tee out/perception/tests.log
```

The runner builds the perception packages, uses separate ROS domains, prints a pass/fail summary, and exits nonzero if a case fails. The full test output is saved in `out/perception/tests.log`; `pipefail` preserves the runner's failure status through `tee`. Use `./src/perception/tests/run_all.sh --list` for names or `--case detector` for one matching case. No separately running container or RViz session is required.

| Scenario | What the suite checks |
| --- | --- |
| Recorded inputs | SQLite integrity, 370 image and 371 cloud messages, frames, formats, QoS, and complete payloads. |
| CameraInfo | It keeps the source header and fisheye parameters. It rejects an image with the wrong size. |
| Detector | A blank image produces empty detections and an annotated image with the same time. Invalid encoding produces no output. |
| Missing LiDAR or detections | An empty selected cloud clears RViz. No unsupported 3D location is published. |
| 3D selection edges | Synthetic person, floor, sparse foreground, and distant wall points test the choice of location. The selected cloud keeps the point data, frame, and stamp. |
| Combined pipeline | A short playback of the real recording produces CameraInfo, detections, finite 3D locations, and valid selected clouds with points. |

The combined case writes `out/perception/pipeline-smoke.json` with topic counts, example locations, and arrival times. Best effort delivery means counts may differ across subscribers. The [test runner](src/perception/tests/run_all.py) lists every case. To measure input timing across the **whole recording**, run these commands in the sourced container:

```bash
mkdir -p out/perception
python3 src/perception/tests/audit_bag.py src/perception/data/rosbag/person_walking > out/perception/bag-audit-current.json
python3 src/perception/tests/playback_probe.py \
  --bag src/perception/data/rosbag/person_walking --rate 0.5 --domain 87 \
  --output out/perception/input-timing-full.json
```

The playback probe is separate from the test suite. It records when input messages arrive and checks complete cloud payloads across the whole recording. The combined test checks the actual output pipeline.

## 4. Architecture and decisions

### Navigation nodes, control, and goal lifecycle

| Part | Responsibility |
| --- | --- |
| [`turtle_nav_interfaces`](src/navigation/turtle_nav_interfaces/) | Builds the public `NavigateTurtle` and internal `ControllerNavigate` actions so both C++ nodes share the same generated types. |
| [`navigation_action_server`](src/navigation/turtle_navigation/src/navigation_action_server.cpp) | Offers `/navigate_turtle`, validates goals, tracks cancellation and replacement, forwards feedback, and decides each public result. |
| [`navigation_controller`](src/navigation/turtle_navigation/src/navigation_controller.cpp) | Reads `/turtle1/pose`, turns and drives toward the target, and is the only publisher of `/turtle1/cmd_vel`. |
| [`navigation.launch.py`](src/navigation/turtle_navigation/launch/navigation.launch.py) | Starts both nodes and optionally Turtlesim and the demo goal. Both nodes read [`navigation.yaml`](src/navigation/turtle_navigation/config/navigation.yaml). |

The public `NavigateTurtle` action takes goal `x` and `y`, reports `current_distance` as feedback, and returns `final_distance` and `message` in its result.

The public server sends motion requests through `/controller/navigate`, a second ROS 2 action. I use it as an **internal convention**. Other ROS participants can still discover and call it. Its result includes `stop_confirmed`, which means the controller published zero velocity before finishing. That does not prove a physical robot has stopped. I kept one velocity publisher so the controller is responsible for stopping motion when a goal is canceled or replaced.

The controller updates its turning and forward motion at 20 Hz. It turns in place when the heading error is large and limits both speeds. It succeeds when the turtle gets within `0.15` position units of the goal. I chose a small tolerance because a controller running at a fixed rate cannot reliably land on one exact coordinate. Final orientation is outside this task. The gains, speed limits, tolerance, pose timeout, coordinate bounds, and server deadlines are in `navigation.yaml`.

Accepting a public goal does not move the turtle immediately. The server waits for the controller, sends it a separate goal, and forwards its feedback. Goal and cancel callbacks return quickly. A timer checks goal state and deadlines. Each node uses one executor thread, so its own state changes follow a defined callback order without extra mutexes.

The server keeps **one active goal** and **at most one pending replacement**. The controller also takes one goal at a time. If several requests arrive, the newest pending request wins:

1. If B arrives while A moves, the server asks the controller to stop A. B waits.
2. After the controller publishes zero velocity and returns A's result, A finishes `ABORTED` with a preemption message. B may start.
3. If C arrives before B starts, B finishes `ABORTED` as superseded and C becomes pending. B never commands motion.

For cancellation, the server normally returns `CANCELED` only after the controller confirms it has stopped. A pending goal can be canceled directly because it has never moved. Near success, the first terminal decision handled by the server wins. Once a cancel is accepted, that goal cannot later become a public success. Dispatch tokens and ROS goal IDs prevent late feedback or results from an older goal from finishing a newer one.

The server rejects NaN, infinity, and targets outside `0.5` to `10.5` for either coordinate. It aborts a goal if the controller cannot start, rejects the request, stops sending feedback, or reports a failure. Each wait for a start, feedback, or stop has a deadline. If the server cannot tell whether motion started or cannot confirm a stop, it rejects new goals until both nodes restart. A client goal already in ROS `CANCELING` stays `CANCELED`, but its result message says when stopping was **not confirmed**. Read the message as well as the status.

### Perception calibration method

[`camera_calibration.py`](src/perception/scripts/camera_calibration.py) finds a 9×7 grid of inner chessboard corners with 20 mm spacing. OpenCV compares the known board positions with the corners it sees in each photo to estimate the lens parameters. I used the fisheye `equidistant` model because this camera has a wide angle lens. I excluded `camera0_20261007162543.png` because its board was visibly warped and the fit assumes a flat board. The remaining **31 of 32** photos go into one fit. The script accepts `--exclude` if a reviewer chooses a different set of photos.

### Perception pipeline and location method

| Component | Responsibility |
| --- | --- |
| [`front_camera_info`](src/perception/front_camera_info/) | Reads the fitted YAML and publishes `sensor_msgs/msg/CameraInfo` on `/m20/camera/front/camera_info` with each image's source timestamp and frame. `robot_state_publisher` publishes the three fixed joints from the supplied sensor URDF. |
| [`human_detector`](src/perception/human_detection_node/human_detection_node/detector.py) | Runs pretrained YOLO26n, filters to person class, and publishes annotated `sensor_msgs/msg/Image` with person boxes and confidence scores on `/m20/camera/front/human/image_annotated` and `vision_msgs/msg/Detection2DArray` on `/m20/camera/front/human/detections`. It preserves image headers and emits an empty array for a processed frame with no people. |
| [`human_locator`](src/perception/human_detection_node/human_detection_node/locator.py) | Keeps sampled LiDAR points, finds the nearest cloud by **header timestamp** within 150 ms, transforms it to the optical frame, and projects its points through the fisheye model. It publishes a 3D median `geometry_msgs/msg/PointStamped` on `/human/location` and the chosen `sensor_msgs/msg/PointCloud2` points on `/human/selected_points`. Both use `base_link_custom` and the image time. |
| [`perception.launch.py`](src/perception/human_detection_node/launch/perception.launch.py) | Starts CameraInfo, static TF, detector, and locator without owning playback. The demo launch adds playback and RViz. |

The detector copies each camera image's header into its detections, so the detection keeps the image's sensor timestamp even though inference finishes later. Each LiDAR cloud has its own sensor timestamp. After allowing late clouds to arrive (see replay timing below), the locator chooses the buffered cloud whose timestamp is closest to the image's. It uses that cloud only if the two sensor timestamps differ by at most 150 ms. Arrival order does not decide the pair, the final images have no cloud within this window and therefore produce no location estimate. The published location keeps the image timestamp.

The locator uses one in every four LiDAR points to reduce the work needed to store and project each cloud. This demonstrates the camera–LiDAR association, but sparse people may lose the few returns needed for an estimate. A future version should filter points to the camera’s visible region, then retain all points that project inside person boxes.

The supplied URDF describes the LiDAR and camera positions relative to the robot, the locator uses those transforms to project points into the image and report locations in `base_link_custom`.

The person box identifies an image area, while LiDAR supplies the depth needed for a 3D estimate. The locator first estimates floor height from nearby LiDAR points and removes floor returns. It then considers points in the central 80% of the box's width and height, reducing interference from objects at the edges. From near to far, it chooses the first 0.5 m depth range along the camera's forward axis that contains at least six sampled points. It stops searching if that range starts more than 1 m behind the nearest candidate point. This favors a nearby group supported by several returns over isolated points or distant background surfaces.

The locator publishes the group's median 3D position as a point on the visible surface, not the person's body centre or ground contact. If no group qualifies, it publishes no `/human/location` and clears the old RViz highlight with an empty selected cloud. These are practical heuristics for the supplied recording, not proof that every chosen point belongs to the person. Synthetic edge cases and recording overlays check the intended behavior, but I have not measured sensitivity to the thresholds.

## 5. Evidence and limitations

### Navigation failure boundary

There is a limit to having one velocity publisher. If the controller dies after sending a nonzero command, the server can detect the missing stop acknowledgment and block new goals. It cannot publish zero velocity itself. Stopping immediately after such a crash would require an independent velocity watchdog or another publisher. That would change the design.

### Calibration fit and review artifacts

I initialized the fisheye fit with a focal length estimated from the image width, the image centre as the principal point, and zero distortion. These are starting values, not measurements of the lens, OpenCV adjusts them using all selected photos in one fit. The fit produced **0.654 px RMS reprojection error**. The saved [CameraInfo YAML](src/perception/front_camera_info/config/camera_calibration.yaml) contains the image size, `equidistant` model, K, four D coefficients, identity R, and P.

The [report](src/perception/results/calibration_report.json) lists accepted and rejected photos and the error for each one. The [corner overlays](src/perception/results/calibration_corners.png) ([middle](src/perception/results/calibration_corners_middle.png), [last](src/perception/results/calibration_corners_last.png)) show the detected corners. Reprojection error measures fit to these photos, not 3D person-location accuracy.

### Projection coverage

The [projection review script](src/perception/scripts/review_projection.py) projects every recorded LiDAR point using the supplied URDF and fisheye calibration. Across all 371 clouds, only **1.22% of points projected inside the image** land in its top third. This explains why a person box in that area may lack enough LiDAR support for a location. The review uses unsampled clouds, so the gap cannot be attributed to the locator's point sampling, the recording does not establish whether the cause is the scan pattern, missing returns from surfaces, or both. The full counts and overlays are in `out/perception/projection-review/`.

### Replay timing

Matching LiDAR clouds often arrive after their camera images during 0.5× bag playback. In a full playback probe, the latest observed matching cloud arrived 1.88 seconds after its image. The locator therefore waits two seconds **after receiving a detection** before choosing the nearest cloud by sensor timestamp. This gives late clouds time to enter its buffer. Each `PointCloud2` arrives as a complete message, the wait is for a matching cloud, not for its points to arrive gradually.

The fixed wait is a practical choice for this recording and playback rate, not a guarantee for other workloads. It also delays every location result, even when the matching cloud is already available. A later version should keep detections and clouds in timestamped buffers, publish as soon as a suitable match can be confirmed, and use a bounded timeout for missing or out-of-order messages. Published locations would still carry the image measurement time. The measurements are in `out/perception/bag-audit-current.json`, `out/perception/input-timing-full.json`, and `out/perception/timing-smoke.json`.

### Remaining validation

Review the full recorded output and compare location estimates with reference 3D positions to measure position error. To check the selection heuristics, hold detections and clouds fixed while varying the box crop, minimum point count, and depth thresholds. On labeled visible, obstructed, sparse, and wall-background examples, compare correct estimates, wrong-surface estimates, misses, and changes in the reported point. This would show whether the choices are stable, without reference positions, it would not establish 3D accuracy.
