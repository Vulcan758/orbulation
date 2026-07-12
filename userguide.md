# orbulation — User Guide

ROS2 (Humble) nodes for running ORB-SLAM3 via the `pyorbslam3` Python
bindings, with a SDK-less ZED 2i driver.

| Node | What it does |
|---|---|
| `orbslam_ros` | ORB-SLAM3 wrapper. All six modes (mono/stereo/rgbd × imu on/off). Publishes odometry, pose, tracking state. |
| `zed_stereo_ros` | ZED 2i driver without the ZED SDK: stereo images over V4L2, 300 Hz IMU + magnetometer/environment via zed-open-capture. |

---

## 0. One-time setup

```bash
# 1. Build the pyorbslam3 bindings (only after changing their C++ source)
cd /home/mahir/orbulating/pyorbslam3
./build.sh
pip install -e .          # only needed once

# 2. Build the ROS package
cd /home/mahir/orbulating/orb_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select orbulation

# 3. In EVERY terminal you use:
source /opt/ros/humble/setup.bash
source /home/mahir/orbulating/orb_ws/install/setup.bash
```

The ORB vocabulary lives at `/home/mahir/orbulating/pyorbslam3/vocabulary/ORBvoc.txt`
and takes **~40 s to load** every time `orbslam_ros` starts. The node prints
`Subscribed; waiting for data` when it is actually ready.

---

## 1. Running against the TUM-VI corridor bag

### With the launch file (easiest)

```bash
ros2 launch orbulation tumvi_bag.launch.py
```

This starts `orbslam_ros` in stereo-inertial mode and automatically plays
`dataset/tum/dataset-corridor4_512_16` after a 45 s delay (vocabulary load).
Useful arguments:

```bash
ros2 launch orbulation tumvi_bag.launch.py visualization:=true   # Pangolin viewer
ros2 launch orbulation tumvi_bag.launch.py rate:=0.5              # slower playback
ros2 launch orbulation tumvi_bag.launch.py bag:=/path/to/bag      # another TUM-VI bag
ros2 launch orbulation tumvi_bag.launch.py play_bag:=false        # play the bag yourself
```

### Manually (two terminals)

```bash
# T1 — the SLAM node
ros2 run orbulation orbslam_ros --ros-args \
  --params-file ~/orbulating/orb_ws/src/orbulation/config/corridor_stereo_imu.yaml

# T2 — wait for "Subscribed; waiting for data" in T1, then:
ros2 bag play ~/orbulating/dataset/tum/dataset-corridor4_512_16
```

### What you should see

- `Tracking state -> OK` within a second of the first frames.
- `IMU initialized (metric scale + gravity available)` after ~3 s.
- A `[status]` line every 5 s: `state=OK fps=~15-20 ok=N/M skipped=0`.
- On Ctrl-C the trajectory is written to
  `/home/mahir/orbulating/orb_ws/traj_corridor_stereo_imu.txt` (EuRoC format:
  `timestamp_ns tx ty tz qx qy qz qw`), plus a `.kf` keyframes-only file.

Reference result on this machine: 1513/1514 frames OK, ATE ≈ 20 cm vs the
bag's mocap segment, scale ≈ 0.95.

---

## 2. Running live with the ZED 2i

### With the launch file (easiest)

```bash
ros2 launch orbulation zed_live.launch.py
# with the Pangolin map viewer:
ros2 launch orbulation zed_live.launch.py visualization:=true
# driver only (no SLAM), e.g. to record a bag:
ros2 launch orbulation zed_live.launch.py slam:=false
```

### Manually (two terminals)

```bash
# T1 — ZED driver (images + IMU)
ros2 run orbulation zed_stereo_ros --ros-args \
  --params-file ~/orbulating/orb_ws/src/orbulation/config/zed_stereo_params.yaml

# T2 — SLAM (same params file configures both nodes)
ros2 run orbulation orbslam_ros --ros-args \
  --params-file ~/orbulating/orb_ws/src/orbulation/config/zed_stereo_params.yaml
```

### Getting stereo-inertial to initialize

The IMU cannot initialize while the camera is static — ORB-SLAM3 prints
`not enough acceleration` and periodically resets the map. That is normal.
Once the SLAM node is ready:

1. Point the camera across the room (nothing closer than ~0.5 m), decent light.
2. Move it **smoothly and continuously** — slow figure-8s, half-meter arcs,
   gentle tilts. Avoid sharp shaking (motion blur kills tracking).
3. `IMU initialized` should appear within a few seconds of starting to move.

Validated on a recorded 90 s handheld session: 1706/1707 frames OK, IMU
initialized at 2.7 s, zero resets.

### Recording a ZED bag for offline work

```bash
ros2 launch orbulation zed_live.launch.py slam:=false
# in another terminal:
ros2 bag record -o my_bag /zed/left/image_raw /zed/right/image_raw /zed/imu/data
```

Note when replaying such a bag into `orbslam_ros`: the driver already applies
the calibrated `image_time_offset` at capture time, so recorded stamps are
correct — replay works as-is.

---

## 3. Outputs

| Topic (default) | Type | Content |
|---|---|---|
| `/orbslam/odometry` | `nav_msgs/Odometry` | camera pose in the SLAM world frame (`odom` → `camera`) |
| `/orbslam/pose` | `geometry_msgs/PoseStamped` | same pose, lighter |
| `/orbslam/tracking_state` | `std_msgs/Int32` | -1 NOT_READY, 0 NO_IMAGES, 1 NOT_INITIALIZED, 2 OK, 3 RECENTLY_LOST, 4 LOST |

```bash
ros2 topic hz /orbslam/odometry
ros2 topic echo /orbslam/pose
```

Odometry is only published while the pose is valid (state OK or
RECENTLY_LOST). In inertial modes the world frame is gravity-aligned and the
scale is metric once the IMU has initialized.

---

## 4. Parameter reference (`orbslam_ros`)

All parameters have defaults and live in the params yaml files
(`config/corridor_stereo_imu.yaml`, `config/zed_stereo_params.yaml`).

| Parameter | Default | Meaning |
|---|---|---|
| `camera_setup` | `stereo` | `monocular` \| `stereo` \| `rgbd` |
| `imu_used` | `true` | adds IMU_ to the sensor mode |
| `vocabulary_path` | pyorbslam3 ORBvoc.txt | ORB vocabulary |
| `config` | TUM-VI stereo-inertial yaml | ORB-SLAM3 settings (calibration, ORB, IMU) |
| `visualization` | `false` | Pangolin viewer (needs a display) |
| `left_cam_topic` / `right_cam_topic` | `/cam0/image_raw` / `/cam1/image_raw` | stereo inputs |
| `mono_cam_topic` | `/cam0/image_raw` | monocular input |
| `rgb_cam_topic` / `depth_cam_topic` | `/camera/rgb/image_raw` / `/camera/depth/image_raw` | rgbd inputs |
| `imu_topic` | `/imu0` | `sensor_msgs/Imu` input |
| `odom_topic` / `pose_topic` / `state_topic` | `/orbslam/...` | outputs |
| `odom_frame` / `base_frame` | `odom` / `camera` | frame ids stamped on outputs |
| `sync_queue_size` / `sync_slop` | `10` / `0.01` | stereo & rgbd pair matching |
| `imu_wait_timeout` | `0.15` | max seconds to wait for IMU coverage per frame |
| `status_period` | `5.0` | seconds between `[status]` log lines |
| `save_trajectory_path` | `""` | if set, EuRoC trajectory written on shutdown |

Key `zed_stereo_ros` parameters: `video_device` (-1 = auto-detect),
`frame_width`/`frame_height` (side-by-side, 1344x376 = VGA per eye), `fps`,
`image_time_offset` (**-0.08, calibrated — don't zero it**), topic names, and
frame ids.

---

## 5. Troubleshooting

**`not enough acceleration` spam (inertial modes)** — camera not moving.
Expected while static; move the camera (see §2).

**Everything stuck in NOT_INITIALIZED with 0 points** — inertial mode resets
the map after tracking loss until IMU init succeeds. Give it smooth motion;
check the images aren't blocked/too dark (`ros2 run rqt_image_view rqt_image_view`).

**ZED images stop / `select() timeout`** — a previous reader was hard-killed
and wedged the UVC stream. The driver now reopens the device automatically
(~10 s); replugging the USB also clears it.

**`ros2 topic echo` on image topics shows nothing** — the images are
best-effort QoS; use `ros2 topic hz` or
`ros2 topic echo --qos-reliability best_effort`.

**Frame timestamp went backward** — you replayed a bag with `--loop`, which
visual-inertial SLAM cannot handle. Restart the node instead.

**Skipped frames (`Empty IMU window` / `No IMU coverage`)** — the IMU stream
isn't reaching the node or is on a different clock than the images. Check
`ros2 topic hz <imu_topic>` and that both stamps are close to wall time.

**Poses look fine but scale/gravity is off (ZED)** — the IMU-camera rotation
in `config/camera_params/zed2i.yaml` was calibrated for this unit
(SN 32957407) on 2026-07-09. If the camera is remounted or swapped, record a
90 s smooth-motion bag (§2) and rerun
`scripts/calibrate_imu_extrinsics.py` to regenerate `IMU.T_b_c1` and the
`image_time_offset`.

**Node won't die on Ctrl-C in scripts** — `ros2 run` wraps the node in a
child process; signal the actual `orbslam_ros` python process, and give it
up to a minute — after loop closures, global bundle adjustment must finish
before shutdown.
