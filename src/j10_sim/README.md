# j10_sim — ArduPilot SITL + Gazebo

Simulation-only package: Gazebo worlds, the indoor ArduPilot parameter set, and the
Phase 1 launch files. Per boundary rule 4 in `docs/ARCHITECTURE.md`, nothing depends on
this package at runtime.

---

## Phase 1 goal

> **Exit criterion:** a `TwistStamped` published on `/j10/cmd_vel_safe` moves the vehicle in
> the commanded body-frame direction, and releasing the command returns it to hover within
> 1 s.

Only `mavlink_bridge_node` and `vehicle_state_node` run. No video, no VLA, no safety filter
— you are the safety filter, and you are publishing the command by hand.

---

## Prerequisites

Three things live outside the ROS workspace and are **not** installed by `rosdep`.

### 1. MAVROS

```bash
sudo apt install ros-humble-mavros ros-humble-mavros-extras
ros2 run mavros install_geographiclib_datasets.sh   # once; needs sudo
```

### 2. Gazebo

`ardupilot_gazebo`'s `main` branch targets Gazebo **Harmonic** (or Garden), not the Fortress
version that ships in the Humble metapackage. Install Harmonic alongside ROS 2:

```bash
sudo apt install lsb-release gnupg curl
curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
sudo apt update && sudo apt install gz-harmonic
```

### 3. ArduPilot + ardupilot_gazebo

```bash
# Flight code and SITL
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot && Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
./waf configure --board sitl && ./waf copter

# Gazebo plugin and the iris model
git clone https://github.com/ArduPilot/ardupilot_gazebo.git ~/ardupilot_gazebo
cd ~/ardupilot_gazebo && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && make -j4
```

Then either export the two paths or pass them as launch arguments every time:

```bash
export ARDUPILOT_DIR=~/ardupilot
export ARDUPILOT_GAZEBO=~/ardupilot_gazebo
```

### Build the workspace

```bash
cd ~/j10_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

## Running it

### One command

```bash
ros2 launch j10_sim sitl.launch.py
```

This stages Gazebo → SITL (after 5 s) → MAVROS and the J10 nodes (after 10 s). The delays
matter: ArduPilot's JSON backend connects *out* to the ArduPilotPlugin and gives up if
Gazebo is not listening yet.

### Four terminals (what you actually want the first time)

Interleaved output from four processes is unreadable, and you will want to restart SITL
without restarting Gazebo.

```bash
# 1 — Gazebo
ros2 launch j10_sim gazebo.launch.py

# 2 — ArduPilot SITL (--console gives you the MAVProxy prompt and prearm messages)
ros2 run j10_sim run_sitl.sh --console

# 3 — MAVROS
ros2 launch j10_sim sitl.launch.py gazebo:=false sitl:=false j10:=false

# 4 — the J10 nodes
ros2 launch j10_mavlink mavlink.launch.py
```

---

## The Phase 1 test

### Step 1 — confirm the link

```bash
ros2 topic echo /j10/vehicle/state --once
```

Look for `connected: true`, a non-empty `mode`, and `staleness_sec` well under `0.1`. If
`connected` is false, MAVROS is not talking to SITL — check that `fcu_url` matches the
`--out` endpoint (`udp:127.0.0.1:14551` by default).

`ekf_healthy` needs a few seconds after SITL starts while EKF3 settles on optical flow and
the rangefinder. If it never goes true, check `EK3_SRC1_*` against `config/sitl_indoor.parm`.

Confirm the setpoint stream is already running, disarmed, at 30 Hz:

```bash
ros2 topic hz /mavros/setpoint_raw/local
```

That stream is never silent by design — ArduPilot's guided failsafe trips on setpoint
silence, so the bridge publishes zeros when it has nothing else to send.

### Step 2 — arm and take off

```bash
ros2 service call /j10/vehicle/arm j10_interfaces/srv/ArmDisarm "{arm: true, force: false}"
ros2 service call /j10/vehicle/takeoff std_srvs/srv/Trigger "{}"
```

The takeoff call blocks until the vehicle reaches 1.5 m (`takeoff_altitude_m`). During that
climb the bridge **stops** publishing setpoints on purpose: ArduPilot runs its own guided
takeoff controller, and a zero-velocity setpoint would override it and leave the vehicle on
the ground.

If arming is refused, the message says why. `EKF is not reporting a usable estimate` means
wait a few more seconds. In SITL only, you can set `allow_force_arm: true` in
`j10_mavlink/config/mavlink.yaml` and pass `force: true`.

### Step 3 — fly it by hand

**The rate matters.** `ros2 topic pub` defaults to 1 Hz, and the bridge treats a command
older than `command_timeout_sec` (300 ms) as stale and decays to hover. At 1 Hz the vehicle
would twitch once per second and stop. Always pass `-r 30`:

```bash
# forward at 0.5 m/s (body +x)
ros2 topic pub -r 30 /j10/cmd_vel_safe geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.5, y: 0.0, z: 0.0}}}"
```

Watch the drone in Gazebo move **toward its own nose**. Then `Ctrl-C` the publisher: it
should come to a hover well inside 1 s (300 ms timeout plus the vehicle's own decel).

Work through the axes — this is what actually proves the frame convention:

| Command | Expected motion in Gazebo |
|---------|---------------------------|
| `linear: {x: 0.5}` | forward, along the nose |
| `linear: {y: 0.5}` | to the vehicle's **left** |
| `linear: {z: 0.3}` | **up** |
| `angular: {z: 0.5}` | yaw **counter-clockwise** seen from above |

All four are ROS body-FLU conventions. If motion is mirrored on y, z and yaw but correct on
x, the FLU/FRD transform is being applied twice — see "Frame conventions" below.

### Step 4 — land and disarm

```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'LAND'}"
ros2 service call /j10/vehicle/arm j10_interfaces/srv/ArmDisarm "{arm: false}"
```

---

## MAVROS plugin id discovery (temporary — remove once resolved)

`config/mavros_apm.yaml` currently ships `plugin_denylist: ['*']`, which blocks every
MAVROS plugin from loading. This is deliberate and temporary, not a mistake: several stock
plugins on at least one observed build independently reuse the relative topic name
`~/status` (`companion_process_status` and `esc_status` confirmed so far), and whichever
one instantiates second crashes the whole node with a duplicate publisher/subscriber. The
plugin id strings a given build actually registers have already disagreed with what
upstream `ros2`-branch source suggests, so guessing them one crash at a time isn't
reliable — the log from the running binary is the only source that can be trusted.

**To collect the real ids:** run MAVROS with this config and watch the log. Nothing should
crash, because nothing is allowed to instantiate:

```bash
ros2 launch j10_sim sitl.launch.py gazebo:=false sitl:=false j10:=false
```

Every line reads `Plugin <id> ignored`. Collect the full list, then replace the
`plugin_denylist` block with:

```yaml
plugin_denylist:
  - '*'
plugin_allowlist:
  - <sys/state plugin id>       # /mavros/state, battery, estimator_status
  - <local_position plugin id>  # /mavros/local_position/pose, velocity_local
  - <setpoint_raw plugin id>    # /mavros/setpoint_raw/local — the bridge's core output
  - <command plugin id>         # arming, takeoff, set_mode
  - <rangefinder plugin id>     # /mavros/rangefinder/rangefinder
```

using the exact strings the log printed. `is_plugin_allowed()` in `mavros_uas.cpp` matches
both lists with `fnmatch`, so a glob (e.g. `sys*`) is fine if it's unambiguous against the
full id list you collected.

With `/mavros/state` alive, confirm the target set actually covers what `vehicle_state_node`
needs:

```bash
ros2 topic list | grep mavros
```

Every topic in the "Topic contract" table in `docs/ARCHITECTURE.md` under `j10_mavlink`
should be present (`px4flow` is intentionally excluded — see the code comment in
`vehicle_state_node.cpp`, not needed until Phase 6).

---

## Frame conventions

The single most common Phase 1 bug is a mirrored axis, so the convention is pinned down
explicitly and unit-tested in `j10_mavlink/test/test_frame_conventions.cpp`.

`/j10/cmd_vel_safe` carries a **body-FLU** twist: x forward, y left, z up, angular.z
counter-clockwise. The bridge writes it into `mavros_msgs/PositionTarget` **unchanged**,
with `coordinate_frame = FRAME_BODY_NED (8)` and `type_mask = 1479` (velocity + yaw_rate,
ignoring position, acceleration and absolute yaw).

The pass-through is correct because MAVROS does the conversion itself. In
`mavros/src/plugins/setpoint_raw.cpp`, the `FRAME_BODY_NED` branch applies
`ftf::transform_frame_baselink_aircraft` (FLU → FRD) to `velocity`, and
`ftf::transform_frame_ned_enu` to `yaw_rate`, before building the MAVLink packet.
Converting in the bridge as well would double-negate y, z and yaw.

If a future MAVROS release removes that transform, set
`body_frame_convention: "frd"` in `j10_mavlink/config/mavlink.yaml` — no code change.

---

## Troubleshooting

| Symptom | Likely cause |
|---------|--------------|
| Gazebo loads an empty room, no drone | `ardupilot_gazebo` not found. Check `ARDUPILOT_GAZEBO` — the launch prints a warning naming the path it tried. |
| SITL exits immediately with a JSON/socket error | Gazebo was not up yet. Start Gazebo first, or raise `sitl_startup_delay`. |
| `/j10/vehicle/state` never appears | The J10 nodes are not running, or `mavlink.launch.py` was loaded into a single-threaded container. It must be `component_container_mt`. |
| `connected: false` forever | `fcu_url` and the SITL `--out` endpoint disagree. |
| Arming refused, `EKF is not reporting a usable estimate` | EKF3 has not settled. Wait ~10 s. If permanent, check `EK3_SRC1_*` and `SIM_FLOW_ENABLE`. |
| Takeoff succeeds, vehicle then sinks | The setpoint stream did not resume. Check for a `takeoff` warning in the bridge log. |
| Drone twitches once per second and stops | `ros2 topic pub` without `-r 30`. |
| Drone moves opposite on y/z/yaw | FLU/FRD applied twice — see "Frame conventions". |
| Arm service call hangs and times out | The bridge is on a single-threaded executor. Use `component_container_mt` or the standalone `mavlink_bridge_node` executable. |
| `mavros_node: error while loading shared libraries: libdiagnostic_updater.so: ...` | A MAVROS system dependency is missing or was left half-installed. `sudo apt install --reinstall ros-humble-diagnostic-updater ros-humble-mavros ros-humble-mavros-extras && sudo ldconfig`, then confirm with `ldd $(ros2 pkg prefix mavros)/lib/mavros/mavros_node \| grep "not found"` (should print nothing). |
| `mavros_node` dies with `create_subscription() ... existing topic name ... incompatible type` then `terminate called after throwing ... invalid allocator` | A plugin collides on startup — the companion-process-status plugin has been observed doing this on some builds. Not a J10 bug; `config/mavros_apm.yaml` denylists it with a `companion*` glob (plugin id strings have differed across MAVROS releases) and otherwise mirrors MAVROS's own stock ArduPilot `plugin_denylist`. If the error names a *different* topic/type after that, find the owning plugin from the type name and add its id (or a glob) to `plugin_denylist` too — `strings $(ros2 pkg prefix mavros_extras)/lib/libmavros_extras_plugins.so \| grep -i <keyword>` on the actual installed `.so` will show the real registered id if it's unclear from upstream source. |

---

## What is deliberately missing

Phase 1 has no safety filter. Nothing bounds the velocity you publish except the bridge's
own wide saturation limits (`absolute_max_linear_mps`, 2.0 m/s), which are defence in depth
and **not** a safety system. `j10_safety` lands in build-order step 3, and Phase 2 is the
phase not to rush.
