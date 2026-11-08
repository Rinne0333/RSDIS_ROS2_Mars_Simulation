# Ingenuity A-to-B mission

Self-contained ROS 2 Jazzy package. Subscribes to `/ingenuity/odometry`; publishes `/ingenuity/top_blades/thrust`, `/ingenuity/bottom_blades/thrust`, `/ingenuity/swashplate_1/joint`, and `/ingenuity/swashplate_2/joint`.

Mission: initial position A -> take off 2 m -> travel B offset (3 m, 0 m) in odometry world XY -> hold 2 s -> land at B.

## Install

Extract directory into `~/RSDIS_ROS2_Mars_Simulation_sync/src/ingenuity_mission`.

```bash
cd ~/RSDIS_ROS2_Mars_Simulation_sync
colcon build --packages-select ingenuity_mission --symlink-install
source install/setup.bash
ros2 run ingenuity_mission mission_node --ros-args -p arm:=true -p goal_dx:=3.0 -p goal_dy:=0.0 -p flight_height:=2.0
```

## Caveats

This is a complete C++ control pipeline, **not validated flight firmware**. No autonomous-flight guarantee: actuator signs, effective thrust, center of mass and model configuration haven't been checked. Initial attitude is saved as reference, which might not be a proper hover attitude. `top_sign` and `bottom_sign` default to +1 and the publisher clamps negative thrust values to zero, so negative signs currently DISABLE that rotor rather than reverse it. Don't run the original navigation_node simultaneously. Use only inside Gazebo. Abort / FAULT publishes zeros while process remains alive, not a hard real-time failsafe. There is no obstacle avoidance, so a straight path can hit terrain. The "yaw" axis is held near the initial quaternion but there is no independent yaw actuator control.

`mass_kg` is currently an informational parameter and doesn't affect force formula: `hover_total` is the actual hover-thrust feedforward. It must be calibrated for the Gazebo model. `max_thrust_per_rotor` clamps 0..35 N per rotor.
