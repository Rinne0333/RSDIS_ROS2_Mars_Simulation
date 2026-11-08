# Mars world + PX4 X500 + Perseverance

The drone is a PX4 `x500_lidar_down` flying in the existing Space ROS Mars world. PX4 holds attitude and position. `x500_control` only sends position setpoints for a back-and-forth scan, then drives Perseverance to whatever the down lidar marks as not terrain.

```text
~/PX4-Autopilot          PX4 firmware and Gazebo models. Not inside the ROS workspace.
~/RSDIS_ROS2_Mars_Simulation/src/px4_msgs   ROS 2 messages for that firmware.
~/RSDIS_ROS2_Mars_Simulation/src/x500_control
~/RSDIS_ROS2_Mars_Simulation/src/spaceros_gz_demos
```

## 1. One-time install

### PX4

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
make px4_sitl gz_x500_mono_cam_down
```

Mars has no real GPS or RC link. In the PX4 console (`pxh>`, press Enter in the SITL terminal) after the first successful spawn:

```text
param set SYS_HAS_BARO 0
param set COM_ARM_WO_GPS 1
param set COM_RC_IN_MODE 4
param set NAV_DLL_ACT 0
param save
```

Ctrl+C once the first simulation window opens. That build is only to compile SITL and fetch the models.

### ROS packages

```bash
cd ~/RSDIS_ROS2_Mars_Simulation/src
git clone https://github.com/PX4/px4_msgs.git

cd ~/RSDIS_ROS2_Mars_Simulation
rm -rf build/px4_msgs install/px4_msgs log/px4_msgs
colcon build --packages-select px4_msgs x500_control
source install/setup.bash
```

Do not pass `--symlink-install` for `px4_msgs`. If the build says `existing path cannot be removed: Is a directory`, run the `rm -rf` line again and rebuild.

`px4_msgs` must be the checkout that matches this PX4 tree. If a message field fails to compile, `cd ~/RSDIS_ROS2_Mars_Simulation/src/px4_msgs && git pull`, then rebuild both packages.

### Micro XRCE-DDS Agent

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

```bash
cd ~/RSDIS_ROS2_Mars_Simulation
colcon build --packages-select spaceros_gz_demos
source install/setup.bash
```

## 2. Every run

Four terminals, in this order.

### **Terminal 1. Mars and Perseverance**

```bash
cd ~/RSDIS_ROS2_Mars_Simulation
source install/setup.bash
ros2 launch spaceros_gz_demos mars.launch.xml
```

Leave it open. Do not start a second Gazebo. Run the simulation.

### **Terminal 2. Ground Truth Perseverance Odometry**

´´'bash
ros2 run ros_gz_bridge parameter_bridge   '/perseverance/ground_truth/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'
´´´ 

### **Terminal 3. Perseverance Navigation Node start**

```bash
cd ~/RSDIS_ROS2_Mars_Simulation
ros2 run perseverance_navigation navigation_node
```


### **Terminal 4. Attach PX4 and spawn the drone**

```bash
cd ~/PX4-Autopilot
pkill -f 'bin/px4' 2>/dev/null || true

PX4_GZ_STANDALONE=1 \
PX4_SYS_AUTOSTART=4014 \
PX4_SIM_MODEL=gz_x500_mono_cam_down \
PX4_GZ_MODEL=x500_mono_cam_down \
PX4_GZ_WORLD=mars \
PX4_GZ_MODEL_POSE="8,-2,3.8,0,0,0" \
make px4_sitl gz_x500_mono_cam_down
```

`4016` is the `x500_lidar_down` airframe.

Wait for `world: mars, model: x500_lidar_down_0` and no `Unable to find uri[x500]`. The `gz_frame_id` line is only a warning. If PX4 says `server already running for instance 0`, run the `pkill` and start it again.

Wait until you get the message `INFO  [commander] Ready for takeoff!`. If it doesn't show up it means the simulation isn't running.

***Debugging:***
`commander arm` and `commander takeoff` in the PX4 shell are the manual check that the vehicle itself can fly. 

### **Terminal 5. Bridge, then the mission**

```bash
MicroXRCEAgent udp4 -p 8888
```

### **Terminal 6. Mission**

Start the agent after PX4 is up.

```bash
cd ~/RSDIS_ROS2_Mars_Simulation
source install/setup.bash
ros2 launch x500_control scan.launch.py
```

A healthy scan log is:

```text
Scan ready: N waypoints
phase -> arm
phase -> offboard
phase -> climb
phase -> scan
```

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
cd ~/RSDIS_ROS2_Mars_Simulation
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
| `px4_msgs` symlink error | `rm -rf build/px4_msgs install/px4_msgs log/px4_msgs` and build again without `--symlink-install` |
| `MicroXRCEAgent: command not found` | Install `https://github.com/eProsima/Micro-XRCE-DDS-Agent.git` and source `/opt/ros/jazzy/setup.bash` |
| `Preflight Fail: barometer` / `No connection to the GCS` | Sensor plugins missing, or the four `param set` lines above were not saved |
| Drone falls over at spawn | Lower Z in `PX4_GZ_MODEL_POSE` and `spawn_z` together |
| `PX4 server already running` | `pkill -f 'bin/px4'` |
