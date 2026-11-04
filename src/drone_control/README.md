# Ingenuity teleop

Build the workspace and source its overlay, then run the Mars simulation:

```bash
colcon build --packages-select spaceros_gz_demos drone_control
source install/setup.bash
ros2 launch spaceros_gz_demos mars.launch.xml
```

For autonomous operation, start the non-interactive flight controller in a
second sourced terminal:

```bash
ros2 run drone_control ingenuity_flight_controller
```

Then start the autonomous area scan in a third sourced terminal:

```bash
ros2 run drone_control ingenuity_area_scan
```

The scanner climbs to the absolute world altitude `scan_altitude` (default 15 m),
waits until vertical speed settles, then flies a serpentine rectangle centered
on the starting x/y position. The conservative shakedown defaults are 10 m by
10 m with 2 m lane spacing. Set dimensions to the known safe terrain bounds for
a full survey, for example:

```bash
ros2 run drone_control ingenuity_area_scan --ros-args \
	-p scan_altitude:=15.0 -p scan_length:=80.0 -p scan_width:=60.0 \
	-p lane_spacing:=4.0
```

`ingenuity_teleop` is manual-only debugging: run it instead of
`ingenuity_flight_controller` when entering commands by hand. The scanner does
not use teleop. Do not run both controller executables simultaneously; both
publish commands to the same rotors and swashplates.

The controller holds the helicopter's position when it first receives
`/ingenuity/odometry`. Gazebo publishes rotor joint states to
`/ingenuity/joint_states`; inspect them with `ros2 topic echo
/ingenuity/joint_states`. The controller log reports signed rotor RPM and yaw
rate. The `valid` flags remain zero until joint-state telemetry arrives. Enter
movement commands in meters along world axes, for
example `x=10` moves 10 m in the positive x direction from the current position,
`y=-2` moves 2 m in the negative y direction, and `z=1` raises the altitude
target by 1 m. These `=` commands are relative moves: repeating `z=3` adds
another 3 m to the current target. `x+=2` and `x-=2` adjust the existing x
target by 2 m. `stop` captures the current x/y while preserving the altitude
target; `hold` captures all three current coordinates. `help` lists commands
and `quit` stops the node.

For a positive z move from the ground, the controller uses 35 N per rotor only
for the first 0.25 m, then follows a bounded climb-speed profile and stabilizes
x/y after clearing the ground. At the target altitude (within 3 cm and with
vertical speed below 0.15 m/s), it captures the current x/y position, like
`stop`, then holds altitude around a 30 N per rotor hover baseline. The mass
estimate defaults to 16 kg and can be overridden with the `mass_kg` ROS
parameter. Maximum thrust is 100 N per rotor.