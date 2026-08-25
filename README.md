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
colcon build
source install/setup.bash
```

Run the tests with `colcon test && colcon test-result --verbose`. A clean run is
**452 tests, 0 errors, 0 failures** (that count includes the linters, which colcon reports
as tests alongside the 354 unit tests).

Two things that will bite you once each, both environment rather than code:

- **`colcon build --symlink-install` after a plain `colcon build`** (or the reverse) fails
  with *"failed to create symbolic link ... because existing path cannot be removed"*. The
  two layouts are not interchangeable in one build tree. Pick one and stick to it; to
  switch, `rm -rf build install log` first.
- **`pytest.missing_result` on the three Python packages** means pytest could not start at
  all. ROS 2 Humble's own plugins and a modern `anyio` bracket the usable pytest range from
  both sides: below 7.0, `anyio`'s plugin fails importing `_pytest.scope`; from 8.0,
  `launch_testing`'s plugin uses a `path` hook argument pytest removed. **pytest 7.x
  satisfies both** — `pip3 install --user "pytest==7.4.4"`. Note the failure is in pytest's
  *startup*, so it reports zero tests rather than failing ones, which reads like the suite
  is empty rather than broken.

## Status

**Every package in the build order (`docs/ARCHITECTURE.md` §9) now exists.**

| Package | Role | Tests |
|---------|------|-------|
| `j10_interfaces` | The message contract everything keys off | — |
| `j10_mavlink` | Sole owner of the FC interface; 30 Hz setpoint stream | 11 |
| `j10_sim` | Gazebo world, ArduPilot SITL, indoor parameter set | — |
| `j10_safety` | The independent guardian — the only node that may veto | 39 |
| `j10_control` | Intent → smooth 30 Hz command, decays to hover | 23 |
| `j10_video` | RTP receiver; capture-time stamping, link health | 40 |
| `j10_telemetry` | Latency percentiles vs. the budget; dataset capture | 41 |
| `j10_vla` | Inference node + Phase 4 scripted backend | 77 |
| `j10_mission` | State machine; owns the autonomy permission | 81 |
| `j10_teleop` | Joystick, deadman, E-stop — top of the arbitration order | 42 |

**354 unit tests, no simulator and no ROS required to run them, milliseconds end to end.**
That is possible because each package keeps its real logic in a pure core with the ROS
wrapper kept thin — the safety envelope, the state machine, the RTP clock and the
percentile maths are all plain C++ or Python.

Phases 3–7 remain: they are bring-up and measurement against real hardware, not new
packages.

**Phase 1 exit criterion: passed in SITL.** A `TwistStamped` of `linear: {x: 1.0}` held for
10 s moved the vehicle **+7.85 m along body-forward** with y and z unchanged (-0.05 m,
-0.01 m), and releasing the command brought it to rest with 1.6 mm of drift — inside the
1 s the criterion allows. Note that arming and takeoff currently need a manual MAVProxy
bring-up on this ArduCopter build; see
[the runbook](src/j10_sim/README.md#if-arming-or-takeoff-hangs--known-mavrosardupilot-48-dev-incompatibility).

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
