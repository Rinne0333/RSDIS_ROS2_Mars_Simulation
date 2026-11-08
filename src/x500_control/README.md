# Mars world + PX4 X500 + Perseverance

The drone is a PX4 `x500_lidar_down`. PX4 holds the hover. This repo only sends position setpoints and then drives Perseverance to whatever the scan marks as not-terrain.

The Martian surface model is a static mesh, pose `40 0 -25`, scale `100 100 100`. It is large and has ridges. Keep the first scans small and centred on the spawn.

## One-time setup

PX4 does **not** live inside the ROS workspace.

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
# deps once, if you have not already:
# bash ./Tools/setup/ubuntu.sh
make px4_sitl gz_x500_lidar_down
```

Inside the jazzy container, `$HOME` is already `.../jazzyDesktop`, so the tree is `$HOME/PX4-Autopilot`.

Build `px4_msgs` into the same workspace (message definitions must match this PX4 checkout):

```bash
cd ~/mars_ws/src
git clone https://github.com/PX4/px4_msgs.git
cd ~/mars_ws
colcon build --packages-select px4_msgs
```

Install the Micro XRCE-DDS Agent so PX4 topics show up in ROS 2 (`MicroXRCEAgent` on `PATH`).

### Gazebo must see the PX4 models

`x500_lidar_down` includes `x500` and `model://LW20`. `load_world.launch.py` used to **replace** `GZ_SIM_RESOURCE_PATH` with only the Space ROS models, which produces:

```text
Unable to find uri[x500]
Unable to find uri[model://LW20]
```

Replace `spaceros_gz_demos/.../launch/load_world.launch.py` with [patches/load_world.launch.py](patches/load_world.launch.py), then:

```bash
cd ~/mars_ws
colcon build --packages-select spaceros_gz_demos
source install/setup.bash
```

`x500_control` is C++ (`rclcpp`). Build it with the rest of the workspace:

```bash
cp -a /path/to/x500_mars/x500_control ~/mars_ws/src/
cd ~/mars_ws
colcon build --packages-select x500_control
source install/setup.bash
```

## Every session

Three terminals. Start them in this order.

**1. Mars world + Perseverance**

```bash
cd ~/mars_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch spaceros_gz_demos mars.launch.xml
```

**2. PX4, attached to that Gazebo (do not start a second world)**

```bash
cd ~/PX4-Autopilot
# only if a previous SITL is wedged:
# pkill -f 'bin/px4'

PX4_GZ_STANDALONE=1 \
PX4_SYS_AUTOSTART=4016 \
PX4_SIM_MODEL=gz_x500_lidar_down \
PX4_GZ_MODEL=x500_lidar_down \
PX4_GZ_WORLD=mars \
PX4_GZ_MODEL_POSE="8,-2,3.8,0,0,0" \
make px4_sitl gz_x500_lidar_down
```

`4016` is the `x500_lidar_down` airframe.

You want `INFO [gz_bridge] world: mars, model: x500_lidar_down_0` and **no** `Unable to find uri[x500]`. The `gz_frame_id` line is a warning only.

**3. ROS 2 bridge to PX4**

```bash
MicroXRCEAgent udp4 -p 8888
ros2 topic list | grep fmu
```

If the topics are versioned (`..._v1`), put those names in `x500_control/config/scan.yaml`.

QGroundControl can arm and take off as a smoke test before the scan node.

## Scan and send the rover

```bash
source ~/mars_ws/install/setup.bash
ros2 launch x500_control scan.launch.py
```

What it does:

1. Streams a position setpoint, arms, switches to Offboard, climbs to `scan_altitude`.
2. Flies a lawnmower (`scan_length` x `scan_width`, `lane_spacing`) in world ENU. Setpoints are converted to PX4 NED about the spawn.
3. Marks a point of interest when the down lidar is suddenly closer than the recent ground (`anomaly_margin`), or when `/target/odometry` (a second rover) is within `target_detect_radius`.
4. Publishes `/x500/poi` (`geometry_msgs/PointStamped`, frame `world`).
5. `rover_investigator` builds a short path inside `safe_*` and drives `/perseverance/cmd_vel` to a standoff `standoff` metres short of the POI.

| Topic | Meaning |
|---|---|
| `/x500/poi` | Detected point, world ENU |
| `/x500/rover_path` | `nav_msgs/Path` the rover is following |
| `/x500/mission_status` | `std_msgs/String` |
| `/target/odometry` | Optional second rover. Not required |
| `/perseverance/odometry`, `/perseverance/cmd_vel` | Investigator rover |

First flight: set `scan_length` and `scan_width` to `8.0` and confirm a hover in QGC or the logs before opening the box. The old crash was the Ingenuity controller tilting itself into a ridge. PX4 holds position; the remaining risk is a scan rectangle that crosses high terrain. Raise `scan_altitude` or shrink the rectangle.

`spawn_x/y/z` in `config/scan.yaml` must match `PX4_GZ_MODEL_POSE`. World X is East, Y is North.

To point the rover at a known object without waiting for lidar, publish once:

```bash
ros2 topic pub --once /x500/poi geometry_msgs/msg/PointStamped \
  "{header: {frame_id: world}, point: {x: 12.0, y: 0.0, z: 0.0}}"
```

Run that with only `rover_investigator` if you do not want the drone to move.


## Whole-map scan, Pathfinder detection and rover path

Nothing in the detection uses simulator knowledge. The drone only has a camera and its own pose, and the terrain grid plays the part of an orbital elevation map.

```text
down camera -> lander_detector -> /x500/detection (pixel)
                                        |
PX4 pose + attitude ------------> poi_locator --(ray vs terrain grid)--> /x500/poi
                                                                           |        |
                                                          offboard_scan holds   rover_investigator plans and drives
```

### Drone model

The stock PX4 `x500_mono_cam_down` already carries a downward camera (1280x960, 1.74 rad), so PX4 needs no patch. Start PX4 with airframe 4014:

```bash
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4014 PX4_SIM_MODEL=gz_x500_mono_cam_down \
PX4_GZ_MODEL=x500_mono_cam_down PX4_GZ_WORLD=mars PX4_GZ_MODEL_POSE="8,-2,3.8,0,0,0" \
make px4_sitl gz_x500_mono_cam_down
```

The lidar of the old model is not used. Check the camera reaches ROS:

```bash
gz topic -l | grep image          # the real gz name; edit mars.launch.xml if it differs
ros2 topic hz /x500/down_camera
```

### Altitude

`martian_surface` peaks at 26.8 m (world Z) near (60, -58), and a 14 m ridge sits on the line from the spawn to the Pathfinder. With `scan_altitude: 0.0` the node flies at the highest terrain in the scan box plus `clearance` (8 m), 34.8 m, as one constant altitude. The log line `Scan ready: ... altitude 34.8 m (highest ground 26.8 m)` confirms it.

### Scan

An S pattern along X over the whole terrain grid, 14 lanes at most 15 m apart (2.8 km). The heading is held east (`scan_yaw`), so the wide side of the image is across the lanes. At 28 m above ground one pixel is 5 cm and the lander is about 30 px wide (100 px at the ridge, 22 px at the lowest ground). Ground coverage is 99.99% even with a 30% margin on the footprint.

### Detector

`lander_detector` uses the fact that the ground is almost a single colour. It takes the image's median chroma (Lab a, b) as background, flags pixels that differ by more than `chroma_thresh`, cleans the mask and keeps the largest blob of plausible size. On crops of the real terrain texture with random shading it gave no false alarms in 180 frames, and on synthetic lander patches it found the object in all frames from 10 to 37 m. That test is synthetic: I have not seen a real render. To tune it on real frames:

```bash
ros2 param set /lander_detector chroma_thresh 14.0
ros2 run rqt_image_view rqt_image_view /x500/detection_debug       # green box on the blob
```

or set `debug_dir` in `scan.yaml` to save frames (hits and every 100th). Lower `chroma_thresh` if the lander is missed, raise it (or `min_blob_area`) if it fires on terrain. If colour is not enough (glare, shadows), the same topic `/x500/detection` can be fed by a trained detector, nothing else changes.

### Locator

`poi_locator` casts the pixel's ray (camera intrinsics, PX4 attitude and position) against the terrain grid. It publishes `/x500/poi` once `confirm_hits` (3) detections land within `confirm_radius` (5 m), which also removes one-frame false alarms. Offline, this recovered 1,272 random camera poses over the Pathfinder to within 0.2 m, the grid resolution. In flight it also depends on timing between image and pose, so expect about a metre of error at speed.

### Terrain grid

`config/terrain_dem.csv` is a 1 m height grid baked from the mesh (regenerate with `tools/make_dem.py` if the surface model changes). It gives 3.81 m at the spawn (PX4 spawn z is 3.8) and 6.57 m at the Pathfinder base (world pose z 6.8).

### Rover path

`rover_investigator` plans on the same grid. It smooths the heights, computes slope, forbids cells steeper than `max_slope_deg` (20) plus `inflate_cells` of margin, and runs A* with a cost that prefers gentle ground. It stops `standoff` metres from the point. For the rover at (0.7, 0) and the Pathfinder at (124, 4):

| `max_slope_deg` | Result |
|---|---|
| 15 | no path |
| 20 | 168 m, steepest leg 18.4 deg |
| 30 | 135 m, steepest leg 27.0 deg |

The straight line is 123 m but crosses the ridge. If the log says `No path under the slope limit`, raise `max_slope_deg`. `/x500/rover_path` carries the path with terrain heights.

Test the rover alone, without flying:

```bash
ros2 topic pub --once /x500/poi geometry_msgs/msg/PointStamped "{header: {frame_id: world}, point: {x: 124.0, y: 4.0, z: 6.6}}"
```

### Build

```bash
cd ~/mars_ws
colcon build --packages-select spaceros_gz_demos x500_control
source install/setup.bash
ros2 launch x500_control scan.launch.py
```

OpenCV and `cv_bridge` come with ROS Jazzy desktop. If `cv_bridge/cv_bridge.hpp` is not found, change the include to `cv_bridge/cv_bridge.h`.
