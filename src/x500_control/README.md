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
