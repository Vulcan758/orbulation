# orbulation — User Guide

ROS2 (Humble) nodes for running ORB-SLAM3 via the `pyorbslam3` Python
bindings, with a SDK-less ZED 2i driver.

| Node | What it does |
|---|---|
| `orbslam_ros` | ORB-SLAM3 wrapper. All six modes (mono/stereo/rgbd × imu on/off). Publishes odometry, pose, tracking state. |
| `zed_stereo_ros` | ZED 2i driver without the ZED SDK: stereo images over V4L2, 300 Hz IMU + magnetometer/environment via zed-open-capture. |

The RealSense D435i needs no driver node of ours — the stock `realsense2_camera`
node is used, configured for SLAM by `d435i_live.launch.py` (§3).

| Launch file | Purpose |
|---|---|
| `tumvi_bag.launch.py` | ORB-SLAM3 against the TUM-VI corridor bag (§1) |
| `zed_live.launch.py` | ZED 2i driver + ORB-SLAM3, live (§2) |
| `d435i_live.launch.py` | RealSense D435i driver + ORB-SLAM3, live (§3) |

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

## 3. Running live with the RealSense D435i

The D435i needs no driver of ours — the stock `realsense2_camera` node does the
job, but it must be configured specifically for SLAM, which is what
`d435i_live.launch.py` handles.

```bash
ros2 launch orbulation d435i_live.launch.py                      # stereo-inertial
ros2 launch orbulation d435i_live.launch.py visualization:=true
ros2 launch orbulation d435i_live.launch.py mode:=rgbd_inertial
ros2 launch orbulation d435i_live.launch.py slam:=false          # driver only
```

The launch file starts the camera driver and `orbslam_ros` together. Four modes,
each with its own params file (`config/d435i_<mode>.yaml`) and its own driver
stream setup:

| `mode:=` | Streams | Emitter |
|---|---|---|
| `stereo_inertial` (default) | infra1 + infra2 + IMU | off |
| `stereo` | infra1 + infra2 | off |
| `rgbd_inertial` | colour + aligned depth + IMU | on |
| `rgbd` | colour + aligned depth | on |

Other arguments: `imu_rate` (200 or 400 Hz, must match `IMU.Frequency` in the
settings file), `initial_reset:=true` (hardware-resets the camera on start — use
it if a previous run was hard-killed and left the device wedged), and
`params_file` to override the config.

### Why the infrared pair rather than colour

`stereo_inertial` is the mode to reach for. The infrared cameras are **global
shutter** and **hardware-rectified**, whereas the colour camera is rolling
shutter, which smears geometry under the exact fast motion visual-inertial SLAM
depends on. The infrared pair also gives true stereo, so scale is metric from
the first frame instead of waiting on IMU initialization.

### Three driver settings that are not optional

Running `ros2 launch realsense2_camera rs_launch.py` and pointing the SLAM node
at it will *not* work. The launch file sets these for you; they matter if you
ever run the driver by hand:

1. **`unite_imu_method:=2`** — the D435i publishes accel and gyro on separate
   topics at different rates. Without this there is no `/camera/camera/imu`
   topic at all and the inertial modes have nothing to subscribe to.
2. **`depth_module.emitter_enabled:=0`** for the infrared modes. The projector
   paints a dot pattern that is fixed in the world but moves with the camera in
   the image, and the front end happily tracks it, wrecking feature matching.
   RGBD modes keep it on because the depth stream needs it.
3. **`align_depth.enable:=true`** for the RGBD modes, and subscribe to
   `/camera/camera/aligned_depth_to_color/image_raw`. The raw
   `/camera/camera/depth/image_rect_raw` lives in the infra1 frame, ~15 mm off
   from the colour image, so every depth lookup would be wrong.

### Manually (two terminals)

```bash
# T1 — camera driver
ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -r __ns:=/camera -r __node:=camera \
  -p enable_infra1:=true -p enable_infra2:=true \
  -p enable_color:=false -p enable_depth:=false \
  -p depth_module.infra_profile:=640x480x30 \
  -p depth_module.emitter_enabled:=0 \
  -p enable_gyro:=true -p enable_accel:=true \
  -p gyro_fps:=200 -p accel_fps:=200 -p unite_imu_method:=2

# T2 — SLAM
ros2 run orbulation orbslam_ros --ros-args \
  --params-file ~/orbulating/orb_ws/src/orbulation/config/d435i_stereo_inertial.yaml
```

### Calibration

The settings files under `config/camera_params/realsense/` hold intrinsics and
IMU extrinsics read straight off **unit SN 215222073406**'s factory EEPROM on
2026-07-21. Unlike the ZED, no hand calibration is needed — librealsense stores
it. If you use a different D435i, re-read it with `rs-enumerate-devices -c` and
update `Camera1.*`/`Camera2.*`, `Stereo.T_c1_c2` and `IMU.T_b_c1`:

- `Camera1.*`/`Camera2.*` ← *Intrinsic of "Infrared 1" / 640x480*
- `Stereo.T_c1_c2` ← negated *Extrinsic from "Infrared 1" To "Infrared 2"* translation
- `IMU.T_b_c1` ← *Extrinsic from "Infrared 1" To "Gyro"* (RGBD: *"Color" To "Gyro"*)

### Verified on this machine

Non-inertial `mode:=stereo`, camera stationary on the desk: state `OK`,
~300 tracked map points, 24 fps, zero skipped frames, and a stationary pose
drifting under 0.2 mm. Stereo-inertial delivers the same tracking but needs
motion before `imu_init` flips true — see below.

### Getting stereo-inertial to initialize

Exactly as with the ZED: **the IMU cannot initialize while the camera is
static.** A stationary D435i in `stereo_inertial` will sit in
`NOT_INITIALIZED` with `ok=0/N`, printing `not enough acceleration` and quietly
resetting the map each time IMU init fails. This is expected and not a fault —
picking the camera up and moving it smoothly clears it within a few seconds. If
you want to confirm the camera and calibration are healthy without moving
anything, run `mode:=stereo` instead, which tracks fine while stationary.

---

## 4. Outputs

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

## 5. Parameter reference (`orbslam_ros`)

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

## 6. Troubleshooting

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

**D435i: no `/camera/camera/imu` topic** — the driver was started without
`unite_imu_method:=2`, so accel and gyro are only published separately on
`/camera/camera/{accel,gyro}/sample`. Inertial modes have no input at all in
that state.

**D435i: tracking is poor / features cling to a dot pattern** — the IR projector
is on. Set `depth_module.emitter_enabled:=0` for the infrared modes.

**D435i: `Failed to set power state` or no frames on start** — a previous run
was hard-killed and left the device wedged. Relaunch with
`initial_reset:=true`, or replug the USB.

**D435i: segfault right after `-Loaded misc parameters`** — the ORB-SLAM3
settings file uses `Camera.type: "Rectified"`. This is an upstream ORB-SLAM3
bug, not a config mistake: `Settings::readCamera2()` only populates
`originalCalib2_` for the `PinHole` and `KannalaBrandt` branches, but
`operator<<(Settings)` dereferences it unconditionally for any stereo sensor, so
printing the settings walks an uninitialised pointer. The stock
`Examples/Stereo-Inertial/RealSense_D435i.yaml` crashes the same way. Our
configs use `Camera.type: "PinHole"` with an explicit `Camera2` block and
`Stereo.T_c1_c2`; since the infrared pair is already hardware-rectified and
distortion is zero, the rectification ORB-SLAM3 then performs is an identity
remap, so nothing about the geometry changes.

**RGBD depth looks offset from colour** — you are subscribed to
`/camera/camera/depth/image_rect_raw`, which sits in the infra1 frame. Use
`/camera/camera/aligned_depth_to_color/image_raw` with
`align_depth.enable:=true`.

**Node won't die on Ctrl-C in scripts** — `ros2 run` wraps the node in a
child process; signal the actual `orbslam_ros` python process, and give it
up to a minute — after loop closures, global bundle adjustment must finish
before shutdown.
