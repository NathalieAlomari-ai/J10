# J10 — Indoor Inspection Drone

Software for an indoor, GPS-denied inspection drone with onboard AI. The repo holds two
tracks that share one airframe:

| Track | Where it runs | What it does | Code |
|---|---|---|---|
| **Onboard** | Raspberry Pi Zero 2W on the drone | Camera → obstacle avoidance + human detection → velocity commands to the flight controller | [`companion/`](companion/) |
| **PC-side** | Ground-station PC over WiFi | Video in → Vision-Language-Action model → safety filter → MAVLink setpoints | [`src/`](src/), [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) |

The onboard track is the one being bench-tested now.

## Hardware

| Part | Component |
|---|---|
| Flight controller | CUAV V7 Nano (ArduPilot) |
| Companion computer | Raspberry Pi Zero 2W, powered by its own 5V/3A BEC |
| Camera | Raspberry Pi Camera Module 3 |
| Indoor positioning | Micoair MTF-01 (optical flow + LiDAR) |
| Range sensor | TFmini-S |
| GPS / compass | Matek SAM-M10Q (M10Q-5883) |
| Motors / ESCs | T-MOTOR F60 PRO V 1950KV, T-MOTOR F45A 6S individual ESCs |
| Propellers | HQProp Ethix S5 5040, inside PA6-CF prop guards |
| Radio | ELRS receiver + handheld transmitter (arming and mode authority) |
| Telemetry | CUAV PW-LINK |
| Audio | 4 × 3 W 8 Ω mini speakers |

## Onboard track — obstacle avoidance and human detection

Three small services on the Pi, no ROS, no ground station in the loop:

```
Pi Camera Module 3 ──► cv_node ──► /dev/shm ──► mavlink_bridge ──UART──► CUAV V7 Nano
                          │
                          └──► human detection ──► log, JSON status, snapshots
```

- **Obstacle avoidance** steers toward the clearest part of the image at 8 Hz.
- **Human detection** (optional, `J10_CV_DETECT_ENABLED=1`) runs a small TFLite model on
  the same frames and reports people it sees. It is an inspection output and does not
  change where the drone flies.
- **`mavlink_bridge`** streams the command to the flight controller at 20 Hz and falls
  back to a zero-velocity hover the moment its input goes missing or stale.

The Pi Zero 2W has 512 MB of RAM and throttles at 80 °C, so the onboard code is built to
run light: Raspberry Pi OS Lite, no Docker, detection capped at 2 Hz on two cores, and a
thermal gate that pauses detection at 75 °C.

Try human detection by itself on the Pi (laptop setup is in the `cv_node` README):

```bash
cd companion/cv_node
python3 -m venv --system-site-packages .venv && source .venv/bin/activate
pip install -e ../j10_shm_protocol && pip install -e . --no-deps
pip install ai-edge-litert
j10-fetch-model
j10-detect picamera2 --max-frames 30 --save-dir ~/detect
```

**Status:** detection is verified on a laptop webcam only. Its speed and temperature on
the Pi Zero 2W have not been measured yet.

Details: [`companion/README.md`](companion/README.md) for the three services and wiring,
[`companion/cv_node/README.md`](companion/cv_node/README.md) for the vision node and
detection settings, [`docs/bench_test_guide.md`](docs/bench_test_guide.md) for the bench
test.

## PC-side track — offboard control

ROS 2 Humble workspace. In this track the drone carries no autonomy: it streams video to a
ground-station PC and accepts MAVLink velocity setpoints. A Vision-Language-Action model on
the PC produces navigation decisions, which pass through an independent safety layer before
reaching the flight controller.

**Full design: [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)**

### Platform

| | |
|---|---|
| Middleware | ROS 2 Humble / Ubuntu 22.04 |
| Flight stack | ArduPilot + MAVROS |
| Companion | Raspberry Pi Zero 2W (video encode + MAVLink routing only) |
| Localization | MTF-01 — optical flow + single-point LiDAR, GPS-denied |
| Video | GStreamer RTP/H.264 over WiFi |
| Latency target | **< 300 ms** glass-to-actuator |

### Architecture in one picture

```
Pi Zero 2W ──RTP/H.264──► video_receiver ──► vla_inference ──► motion_controller
                                                                      │
        ArduPilot ◄── MAVROS ◄── mavlink_bridge ◄── safety_filter ◄────┘
```

The VLA runs at 5–10 Hz and emits *semantic intents*. The motion controller, safety filter,
and MAVLink bridge run at 30 Hz and guarantee the flight controller always has a fresh,
bounded command. **On loss of any input the fast path decays to hover — never to the last
command.**

### Packages

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

### Build

```bash
mkdir -p ~/j10_ws/src && cd ~/j10_ws
git clone https://github.com/NathalieAlomari-ai/J10.git .
vcs import src < j10.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### Status

Phase 0 of 7 — foundation. `j10_interfaces` defines the contract; node packages land in
build order (see `docs/ARCHITECTURE.md` §9).

## Safety

This system commands a real aircraft. Two rules are not negotiable:

1. **The safety filter is independent of the model.** It must be testable, and tested, with
   no simulator, no MAVROS, and no GPU.
2. **Props off until Phase 7.** Hardware-in-the-loop testing happens with the propellers
   physically removed.
