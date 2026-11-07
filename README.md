# Mars world + PX4 X500 + Perseverance

The drone is a PX4 `x500_lidar_down` flying in the existing Space ROS Mars world. PX4 holds attitude and position. `x500_control` only sends position setpoints for a back-and-forth scan, then drives Perseverance to whatever the down lidar marks as not terrain.

Inside the Jazzy container, `$HOME` is `/home/rinne/containers/jazzyDesktop`.

```text
~/PX4-Autopilot          PX4 firmware and Gazebo models. Not inside the ROS workspace.
~/mars_ws/src/px4_msgs   ROS 2 messages for that firmware.
~/mars_ws/src/x500_control
~/mars_ws/src/spaceros_gz_demos
```

The Martian surface mesh is posed at `40 0 -25`, rolled 90 degrees, and scaled by 100. It has ridges. The scan below is an 80 m box centered on the spawn at 30 m altitude, which is the full patch around the vehicles, not a flight into the mesh.

## 1. One-time install

### PX4

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
make px4_sitl gz_x500_lidar_down
```

Ctrl+C once the first simulation window opens. That build is only to compile SITL and fetch the models.

### ROS packages

```bash
cd ~/mars_ws/src
git clone https://github.com/PX4/px4_msgs.git
# copy x500_control into ~/mars_ws/src/ as well

cd ~/mars_ws
rm -rf build/px4_msgs install/px4_msgs log/px4_msgs
colcon build --packages-select px4_msgs x500_control
source install/setup.bash
```

Do not pass `--symlink-install` for `px4_msgs`. If the build says `existing path cannot be removed: Is a directory`, run the `rm -rf` line again and rebuild.

`px4_msgs` must be the checkout that matches this PX4 tree. If a message field fails to compile, `cd ~/mars_ws/src/px4_msgs && git pull`, then rebuild both packages.

### Micro XRCE-DDS Agent

```bash
sudo apt update
sudo apt install ros-jazzy-micro-xrce-dds-agent
```

If that package is not in apt:

```bash
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake .. && make -j$(nproc)
sudo make install && sudo ldconfig
```

### Gazebo must see the PX4 models

`x500_lidar_down` includes `x500` and `model://LW20`. The stock `load_world.launch.py` replaces `GZ_SIM_RESOURCE_PATH` with only the Mars models, which produces `Unable to find uri[x500]`.

Replace `~/mars_ws/src/spaceros_gz_demos/spaceros_gz_demos/launch/load_world.launch.py` with [patches/load_world.launch.py](patches/load_world.launch.py). The call is `os.path.join`, not `os.pathz`.

`mars.launch.xml` does not need a `<set_env>` block. It must stay valid XML: one `<launch>` root, the world include, then one `parameter_bridge` node whose `args="..."` quote is closed before the `<remap>` list. A second `<include>` inside that node breaks the file.

```bash
cd ~/mars_ws
colcon build --packages-select spaceros_gz_demos
source install/setup.bash
```

The Mars world also needs the sensor plugins PX4 expects (IMU, barometer, magnetometer, GPS, lidar). Without them the vehicle spawns, tips over, and PX4 reports `barometer 0 missing`.

### Arming, once per airframe

Mars has no real GPS or RC link. In the PX4 console (`pxh>`, press Enter in the SITL terminal) after the first successful spawn:

```text
param set SYS_HAS_BARO 0
param set COM_ARM_WO_GPS 1
param set COM_RC_IN_MODE 4
param set NAV_DLL_ACT 0
param save
```

Skip `SYS_HAS_BARO 0` if a barometer is actually publishing. `COM_RC_IN_MODE 4` removes the "No connection to the GCS" arming failure. Opening QGroundControl is the alternative.

`x500_control` must not wait for `xy_valid`. On this world that flag stays false. In `src/offboard_scan.cpp`, `tick()` only checks height:

```cpp
if (!pos_ || !pos_->z_valid) {return;}
```

This firmware publishes versioned status topics. `config/scan.yaml` must name the topics that `ros2 topic list | grep fmu` actually prints. The set that matches this SITL is:

```yaml
vehicle_local_position: /fmu/out/vehicle_local_position_v1
vehicle_status: /fmu/out/vehicle_status_v5
vehicle_command: /fmu/in/vehicle_command
```

`ros2 topic info /fmu/in/vehicle_command` must show a subscriber while PX4 is running. If the count is 0, use the `/fmu/in/vehicle_command...` name from `ros2 topic list`.

## 2. Every run

Three terminals, in this order. Source ROS in each one.

```bash
source /opt/ros/jazzy/setup.bash
source ~/mars_ws/install/setup.bash
```

**Terminal 1. Mars and Perseverance**

```bash
cd ~/mars_ws
ros2 launch spaceros_gz_demos mars.launch.xml
```

Leave it open. Do not start a second Gazebo.

**Terminal 2. Attach PX4 and spawn the drone**

```bash
cd ~/PX4-Autopilot
pkill -f 'bin/px4' 2>/dev/null || true

PX4_GZ_STANDALONE=1 \
PX4_SYS_AUTOSTART=4016 \
PX4_SIM_MODEL=gz_x500_lidar_down \
PX4_GZ_MODEL=x500_lidar_down \
PX4_GZ_WORLD=mars \
PX4_GZ_MODEL_POSE="8,-2,3.8,0,0,0" \
make px4_sitl gz_x500_lidar_down
```

`4016` is the `x500_lidar_down` airframe.

Wait for `world: mars, model: x500_lidar_down_0` and no `Unable to find uri[x500]`. The `gz_frame_id` line is only a warning. If PX4 says `server already running for instance 0`, run the `pkill` and start it again.

If the drone falls over on spawn, lower only the Z in that pose (try `3.5`, then `3.3`) and set the same value as `spawn_z` in `scan.yaml`.

**Terminal 3. Bridge, then the mission**

```bash
MicroXRCEAgent udp4 -p 8888
```

Start the agent after PX4 is up. In another terminal:

```bash
ros2 topic echo /fmu/out/vehicle_local_position_v1 --once
ros2 launch x500_control scan.launch.py
```

The echo must print a pose before the launch is useful. A healthy scan log is:

```text
Scan ready: N waypoints
phase -> arm
phase -> offboard
phase -> climb
phase -> scan
```

`commander arm` and `commander takeoff` in the PX4 shell are the manual check that the vehicle itself can fly. The scan node uses the same arm command, but it only notices success on `vehicle_status_v5`.

## 3. What the mission does

1. Streams a position setpoint, arms, switches to Offboard, and climbs to `scan_altitude`.
2. Flies a lawnmower. Lanes run along world X, then it steps in Y and comes back. World X is East and Y is North. Setpoints are converted to PX4 NED about the spawn.
3. Marks a point when the down lidar is suddenly closer than the recent ground (`anomaly_margin`), or when `/target/odometry` is within `target_detect_radius`.
4. Publishes `/x500/poi` (`geometry_msgs/PointStamped`, frame `world`).
5. Drives `/perseverance/cmd_vel` along `/x500/rover_path`, staying inside `safe_*`, and stops `standoff` metres short of the point.

| Topic | Meaning |
|---|---|
| `/x500/poi` | Detected point, world ENU |
| `/x500/rover_path` | Path the rover is following |
| `/x500/mission_status` | Short status string |
| `/target/odometry` | Optional second rover. Not required |
| `/perseverance/odometry`, `/perseverance/cmd_vel` | Investigator rover |

Default survey in `config/scan.yaml`:

```yaml
scan_altitude: 30.0
scan_length: 80.0
scan_width: 80.0
lane_spacing: 10.0
```

With the spawn at `(8, -2)` that is world X from `-32` to `48` and Y from `-42` to `38`. The matching rover box is `safe_x_min/max = -32/48` and `safe_y_min/max = -42/38`.

After any edit to `scan.yaml` or the C++ sources:

```bash
cd ~/mars_ws
colcon build --packages-select x500_control
source install/setup.bash
ros2 launch x500_control scan.launch.py
```

| Key | Effect |
|---|---|
| `spawn_x`, `spawn_y`, `spawn_z` | Must match `PX4_GZ_MODEL_POSE` |
| `scan_altitude` | World Z held during the survey |
| `scan_length`, `scan_width` | Rectangle centered on the spawn. Length is along X |
| `lane_spacing` | Metres between passes. Smaller takes longer |
| `waypoint_radius` | How close it gets to a corner before turning |
| `anomaly_margin` | Metres closer than the ground median before a hit counts |
| `anomaly_hits` | Consecutive close returns required |
| `stop_on_detect` | `false` finishes the pattern and keeps the strongest hit. `true` hovers at the first hit |
| `target_detect_radius` | How close the drone must fly to `/target/odometry` |
| `standoff` | How far short of the point the rover stops |
| `rover_speed`, `rover_yaw_gain` | Drive speed and turn rate |
| `safe_*` | Rover is not allowed outside this world-ENU box |

To send the rover without flying:

```bash
ros2 topic pub --once /x500/poi geometry_msgs/msg/PointStamped \
  "{header: {frame_id: world}, point: {x: 12.0, y: 0.0, z: 0.0}}"
```

## 4. When it fails

| Symptom | Fix |
|---|---|
| `mars.launch.xml` mismatched tag | The bridge `args="` quote was left open, or an `<include>` was pasted inside the `<node>` |
| `px4_msgs` symlink error | `rm -rf build/px4_msgs install/px4_msgs log/px4_msgs` and build again without `--symlink-install` |
| `MicroXRCEAgent: command not found` | Install `ros-jazzy-micro-xrce-dds-agent` and source `/opt/ros/jazzy/setup.bash` |
| Stuck at `Scan ready` | `xy_valid` is false, or the local-position topic name is wrong. Echo `vehicle_local_position_v1` |
| `arm command sent` forever | Status topic is not `vehicle_status_v5`, or `/fmu/in/vehicle_command` has no subscriber. PX4 prints nothing when the command never arrives |
| `Preflight Fail: barometer` / `No connection to the GCS` | Sensor plugins missing, or the four `param set` lines above were not saved |
| Drone falls over at spawn | Lower Z in `PX4_GZ_MODEL_POSE` and `spawn_z` together |
| `PX4 server already running` | `pkill -f 'bin/px4'` |
