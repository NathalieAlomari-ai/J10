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

## GPS-denied EKF: two paths

Two ways to get `ekf_healthy: true` indoors. Use whichever unblocks you fastest, then
switch to the other before hardware.

**`--extnav` — simulated external nav (fast path).** ArduPilot SITL can synthesise an
external-nav MAVLink stream from its own ground truth, so `run_sitl.sh --extnav` gets a
working EKF with no extra ROS node and no MAVROS vision plugin:

```bash
ros2 run j10_sim run_sitl.sh --extnav --console
# or: ros2 launch j10_sim sitl.launch.py extnav:=true
```

This loads `config/sitl_extnav.parm` and attaches ArduPilot's simulated device on
SERIAL5. If the EKF never reports a position on this path, the two values most likely to
have drifted between ArduPilot releases are `VISO_TYPE` and `SIM_VICON_TMASK` — both are
flagged `VERIFY` in the `.parm` file's comments; read them from the running binary with
`param show VISO_TYPE` / `param show SIM_VICON*` rather than trusting the shipped numbers.

**Optical flow (`--extnav` omitted, the default).** Loads `config/sitl_indoor.parm`, and is
the hardware-faithful path toward the real MTF-01. `ARMING_CHECK` is now `1` by default —
it was `0`, which suppressed the exact PreArm message that names which estimator flag is
missing. If arming is refused, read that message before changing any parameter:

```bash
# in the MAVProxy console (run_sitl.sh --console)
arm throttle
```

then check the estimator bits directly:

```bash
ros2 topic echo /mavros/estimator_status --once
ros2 topic echo /mavros/mavros/rangefinder --once
```

`vehicle_state_node` now logs exactly which flags are missing (`attitude`,
`velocity_horiz`, `pos_horiz_rel`, `pos_vert_abs`, `!accel_error`) whenever
`ekf_healthy` is false, instead of only the aggregated boolean.

## The Phase 1 test

### Step 1 — confirm the link

```bash
ros2 topic echo /j10/vehicle/state --once
```

Look for `connected: true`, a non-empty `mode`, and `staleness_sec` well under `0.1`. If
`connected` is false, MAVROS is not talking to SITL — check that `fcu_url` matches the
`--out` endpoint (`udp:127.0.0.1:14551` by default).

`ekf_healthy` needs a few seconds after SITL starts while EKF3 settles. If it never goes
true, the node's own log now names which flag is missing — see "GPS-denied EKF" above for
where to look next on each path.

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
wait a few more seconds — or check `ros2 topic echo /mavros/estimator_status --once`,
since `vehicle_state_node` logs the specific missing flag. In SITL only, you can set
`allow_force_arm: true` in `j10_mavlink/config/mavlink.yaml` and pass `force: true`.

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

## Link loss: proving the KPI, not assuming it

Task A: *"Link-loss safety: stop commands on stream drop -> FC failsafe takes over"*, KPI
*"Commands stop <= 1 s after link loss; FC failsafe verified"*. `mavlink_bridge_node` now
has two different responses depending on WHAT went silent, documented in the class comment
in `mavlink_bridge_node.hpp` — the short version: the model being slow keeps the stream
alive with zeros (ArduPilot's guided failsafe must not trip on that), but
`/j10/cmd_vel_safe` itself going silent for `stream_stop_timeout_sec` means the PC-side
chain is gone, and the bridge stops publishing entirely so the flight controller's own
`FS_GCS_ENABLE` failsafe takes the vehicle. The stop latches; resume explicitly via
`/j10/vehicle/resume_stream` once the cause is understood.

This requires `FS_GCS_ENABLE 1` on the flight controller, which both shipped `.parm` files
now set (previously `0`, which made the KPI undemonstrable — there was no failsafe to hand
over to). Prove it end to end, without arming, with:

```bash
ros2 run j10_sim link_loss_test.py --failsafe-mode LAND
```

It streams `/j10/cmd_vel_safe`, stops, and reports both halves of the KPI: how long the
bridge kept publishing after the cut, and whether `/j10/vehicle/state.mode` subsequently
changed. Armed and in the air, watch it directly:

```bash
ros2 service call /j10/vehicle/stop_stream std_srvs/srv/Trigger "{}"
ros2 topic echo /j10/vehicle/state   # mode should move toward LAND within FS_GCS_TIMEOUT
ros2 service call /j10/vehicle/resume_stream std_srvs/srv/Trigger "{}"   # only once GUIDED
```

`resume_stream` refuses while the vehicle is armed and not back in GUIDED — the flight
controller must be handed control back explicitly (`/j10/vehicle/set_guided`) before
setpoints resume underneath it, so a resume can never fight an in-progress landing.

## Measuring the latency KPI

```bash
ros2 run j10_sim latency_probe.py --duration 30
```

Reports p50/p95/p99 for `/j10/cmd_vel_safe` → setpoint published (the bridge's own
contribution, budget 20 ms) and → measured velocity response (reaction time, not part of
the budget). This is the command-leg half of the 300 ms end-to-end KPI; the frame-capture
half needs `j10_video` (Phase 3).

## MAVROS plugin allowlist — why it's an allowlist, not a denylist

`config/mavros_apm.yaml` loads only five MAVROS plugins (`sys_status`, `command`,
`local_position`, `setpoint_raw`, `rangefinder`) via `plugin_allowlist`, with everything
else blocked by `plugin_denylist: ['*']`. This isn't minimalism for its own sake — several
stock MAVROS plugins turned out to independently reuse the relative topic name `~/status`
(`companion_process_status` and `esc_status` both confirmed crashing `mavros_node` on
startup with a duplicate publisher/subscriber on that name), so denylisting the offenders
one crash at a time proved unreliable. The five above are exactly what Phase 1 needs and
nothing more, which sidesteps the whole class of collision rather than chasing it.

If you ever need to change this list (a later phase needs another plugin, e.g. `imu` or
`sys_time`), don't guess the id string from upstream source — it's already disagreed with
what this exact installed build registers more than once while this config was built
(`command`/`sys_status` here vs. `cmd`/`sys` in the `ros2`-branch source read at the time).
Get the authoritative list straight from the binary instead: temporarily set
`plugin_denylist: ['*']` with no `plugin_allowlist`, which blocks every plugin from
instantiating (so nothing can collide and `mavros_node` comes up clean), and watch the log
— every candidate prints `Plugin <id> ignored` with its real id. Add what you need to
`plugin_allowlist` from that list, then restore the denylist-all line.

Once MAVROS is up, confirm the topic set actually covers what `vehicle_state_node` needs:

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
| Arming refused, `EKF is not reporting a usable estimate` | EKF3 has not settled. Wait ~10 s. If permanent, `vehicle_state_node`'s log now names the missing flag; on the flow path check `EK3_SRC1_*`, `SIM_FLOW_ENABLE`, and that the rangefinder is actually reporting (`RNGFND1_TYPE 1` + `RNGFND1_PIN`, not `RNGFND1_TYPE 100` alone). On `--extnav`, check `VISO_TYPE` and `SIM_VICON_TMASK`. |
| Setpoints stop within a second and never come back, vehicle was never even commanded | This is the link-loss stop working as designed on a stream that never started — `handleArmDisarm` requires `setpoints_published_ > 0`, and the stop watchdog needs `cmd_ever_received_` too, but if you're seeing this outside that window check `stream_stop_timeout_sec` isn't set shorter than your publish rate's period. |
| `link_loss_test.py` reports "FC failsafe verified: FAIL" | `FS_GCS_ENABLE` is 0, or the vehicle was disarmed (ArduPilot does not run the GCS failsafe while disarmed) — the script does not arm the vehicle on purpose, so run it against an already-armed SITL for a real check, or read the two halves separately: the "commands stop" half does not need arming. |
| Takeoff succeeds, vehicle then sinks | The setpoint stream did not resume. Check for a `takeoff` warning in the bridge log. |
| Drone twitches once per second and stops | `ros2 topic pub` without `-r 30`. |
| `/j10/vehicle/state` shows `connected: true` but `pose`/`velocity` stay all-zero with a zero timestamp | Wrong MAVROS source topic. MAVROS is not uniform: `~/`-prefixed plugin topics (`local_position`, `rangefinder`, `setpoint_raw`) resolve to `/mavros/mavros/*`, while `sys_status` uses plain relative names at `/mavros/*`. All source topics are parameters in `j10_mavlink/config/mavlink.yaml`. Confirm a *publisher* exists with `ros2 topic info /mavros/mavros/pose` — a topic can appear in `ros2 topic list` with only our subscriber on it. |
| Vehicle arms and takes off but ignores `/j10/cmd_vel_safe` entirely | The bridge is publishing setpoints to a topic nothing subscribes to. This fails silently. Check `ros2 topic info /mavros/mavros/local` reports `Subscription count: 1`; if not, fix `setpoint_topic` in `j10_mavlink/config/mavlink.yaml`. |
| Drone moves opposite on y/z/yaw | FLU/FRD applied twice — see "Frame conventions". |
| Arm service call hangs and times out | The bridge is on a single-threaded executor. Use `component_container_mt` or the standalone `mavlink_bridge_node` executable. |
| `mavros_node: error while loading shared libraries: libdiagnostic_updater.so: ...` | A MAVROS system dependency is missing or was left half-installed. `sudo apt install --reinstall ros-humble-diagnostic-updater ros-humble-mavros ros-humble-mavros-extras && sudo ldconfig`, then confirm with `ldd $(ros2 pkg prefix mavros)/lib/mavros/mavros_node \| grep "not found"` (should print nothing). |
| `mavros_node` dies with `create_publisher()`/`create_subscription() ... existing topic name ... incompatible type` then `terminate called after throwing ... invalid allocator` | Two different stock plugins are colliding on the same relative topic name (seen with `companion_process_status` and `esc_status`, both reusing `~/status`) — not a J10 bug. `config/mavros_apm.yaml` already works around the whole class of this by loading only five plugins via `plugin_allowlist` rather than denylisting offenders one at a time; if you've added a plugin back to that list and hit this, see "MAVROS plugin allowlist" above for how to find the real id from the running binary rather than guessing from source. |

---

## What is deliberately missing

Phase 1 has no safety filter. Nothing bounds the velocity you publish except the bridge's
own wide saturation limits (`absolute_max_linear_mps`, 2.0 m/s), which are defence in depth
and **not** a safety system. `j10_safety` lands in build-order step 3, and Phase 2 is the
phase not to rush.

The video pipeline, the VLA node, and the motion controller do not exist yet — this
package proves the flight-controller half of the architecture (arm, hover, envelope,
link-loss), not the full task A pipeline. Only `j10_safety`'s side of the envelope has
landed; `j10_video`, `j10_vla`, `j10_control` and the round-trip telemetry link are
unstarted.
