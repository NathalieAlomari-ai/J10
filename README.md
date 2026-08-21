# J10 — Indoor UAV, PC-Side Offboard Control

ROS 2 Humble workspace for an indoor autonomous UAV. The drone carries no autonomy: it
streams video to a ground-station PC and accepts MAVLink velocity setpoints. A
Vision-Language-Action model on the PC produces navigation decisions, which pass through an
independent safety layer before reaching the flight controller.

**Full design: [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)**

## Platform

| | |
|---|---|
| Middleware | ROS 2 Humble / Ubuntu 22.04 |
| Flight stack | ArduPilot + MAVROS |
| Companion | Raspberry Pi Zero 2W (video encode + MAVLink routing only) |
| Localization | MTF-01 — optical flow + single-point LiDAR, GPS-denied |
| Video | GStreamer RTP/H.264 over WiFi |
| Latency target | **< 300 ms** glass-to-actuator |

## Architecture in one picture

```
Pi Zero 2W ──RTP/H.264──► video_receiver ──► vla_inference ──► motion_controller
                                                                      │
        ArduPilot ◄── MAVROS ◄── mavlink_bridge ◄── safety_filter ◄────┘
```

The VLA runs at 5–10 Hz and emits *semantic intents*. The motion controller, safety filter,
and MAVLink bridge run at 30 Hz and guarantee the flight controller always has a fresh,
bounded command. **On loss of any input the fast path decays to hover — never to the last
command.**

## Packages

| Package | Lang | Purpose |
|---------|------|---------|
| `j10_interfaces` | — | Shared message/service contract |
| `j10_video` | C++ | GStreamer receiver → `sensor_msgs/Image` |
| `j10_vla` | Python | VLA inference → `NavIntent` |
| `j10_control` | C++ | Intent → smoothed body-frame velocity |
| `j10_safety` | C++ | Independent limits, geofence, arbitration, E-stop |
| `j10_mavlink` | C++ | MAVROS bridge + aggregated vehicle state |
| `j10_mission` | Python | Mission state machine |
| `j10_teleop` | C++ | Joystick override + deadman |
| `j10_telemetry` | Python | Latency monitoring + bag recording |
| `j10_bringup` | — | Launch files and parameters |
| `j10_sim` | — | Gazebo worlds + ArduPilot SITL |

Only `j10_mavlink` may subscribe to `/mavros/*`. Everything else reads
`/j10/vehicle/state`.

## Build

```bash
mkdir -p ~/j10_ws/src && cd ~/j10_ws
git clone https://github.com/NathalieAlomari-ai/J10.git .
vcs import src < j10.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Status

Phase 1–2 of 7. `j10_interfaces` defines the contract, `j10_mavlink` + `j10_sim` bring up
ArduCopter SITL in Gazebo with a hand-published velocity command, `j10_control` shapes
intents into a smooth 30 Hz command, and `j10_safety` enforces the envelope on every
command before it reaches the flight controller — **62 unit tests, no simulator required,
milliseconds to run.** Remaining packages land in build order (see
`docs/ARCHITECTURE.md` §9).

**Runbook: [`src/j10_sim/README.md`](src/j10_sim/README.md)** — prerequisites, bring-up, and
the Phase 1 exit test.

```bash
ros2 launch j10_sim sitl.launch.py
ros2 service call /j10/vehicle/arm j10_interfaces/srv/ArmDisarm "{arm: true}"
ros2 service call /j10/vehicle/takeoff std_srvs/srv/Trigger "{}"

# fly it — note -r 30; the bridge decays to hover 300 ms after the last command
ros2 topic pub -r 30 /j10/cmd_vel_safe geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.5}}}"
```

`/j10/cmd_vel_safe` is a **body-FLU** twist: x forward, y left, z up, angular.z
counter-clockwise.

## Safety

This system commands a real aircraft. Two rules are not negotiable:

1. **The safety filter is independent of the model.** It must be testable, and tested, with
   no simulator, no MAVROS, and no GPU.
2. **Props off until Phase 7.** Hardware-in-the-loop testing happens with the propellers
   physically removed.
