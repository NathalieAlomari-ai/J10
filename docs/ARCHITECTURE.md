# J10 — PC-Side Offboard Control Architecture

**Target platform:** ROS 2 Humble / Ubuntu 22.04
**Vehicle:** Raspberry Pi Zero 2W companion + ArduPilot flight controller + MTF-01 (optical flow + single-point LiDAR)
**Constraint:** end-to-end glass-to-actuator latency under **300 ms**

---

## 1. Overview

J10 runs a **PC-side Offboard Control** architecture. The drone carries no autonomy: it
streams H.264 video to a ground-station PC and accepts MAVLink velocity setpoints. All
perception, decision-making, and safety enforcement happen on the PC.

```
                  ┌──────────────────── DRONE ────────────────────┐
                  │  Pi Zero 2W          ArduPilot FC             │
                  │  camera + encoder    MTF-01 (flow + LiDAR)    │
                  └───────┬───────────────────────▲───────────────┘
                          │ RTP/H.264/UDP         │ MAVLink/UDP
                          ▼                       │
                  ┌───────────────── GROUND PC ───┴───────────────┐
                  │  video → VLA → controller → safety → MAVROS   │
                  └───────────────────────────────────────────────┘
```

### Platform decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Flight stack | **ArduPilot + MAVROS** | Native MTF-01 support; mature EKF3 optical-flow fusion for GPS-denied indoor flight |
| VLA runtime | **Local open-weights model on GPU** | Only option with predictable latency inside a 300 ms budget |
| Video transport | **GStreamer RTP/H.264** | ~60–100 ms glass-to-ROS; tolerates WiFi loss; no ROS 2 stack on the Pi Zero 2W |
| Language split | **C++ hot path, Python VLA** | Deterministic control path with intra-process zero-copy; fast iteration on the model side |

---

## 2. The controlling design decision: a two-rate cascade

A VLA model does not run at control rate. Even a well-optimized local checkpoint lands at
5–10 Hz, and its output is semantic ("go through the doorway"), not a servo command. Driving
the flight controller directly from it would leave a 100–200 ms hole in the command stream
every cycle — and ArduPilot's guided-mode failsafe stops the vehicle after ~3 s of setpoint
silence.

So the system splits into two rates:

```
VLA (5-10 Hz, semantic intent)
  └─► motion controller (30 Hz, smooth velocity)
        └─► safety filter (30 Hz, independent veto)
              └─► MAVROS bridge (30 Hz, never-silent setpoint stream)
```

The slow layer decides *where to go*. The fast layer guarantees the vehicle always has a
fresh, bounded, safe command — including when the VLA stalls, returns garbage, or the network
drops.

> **The fast layer's default output is zero velocity (hover), not "last command."**
> Repeating a stale command on loss of input is how offboard systems fly into walls.

---

## 3. Workspace & package structure

```
j10_ws/
├── src/
│   ├── j10_interfaces/      # ament_cmake  — all custom .msg/.srv/.action
│   ├── j10_bringup/         # ament_cmake  — launch files, YAML params, RViz configs
│   ├── j10_video/           # ament_cmake  — GStreamer receiver (C++)
│   ├── j10_control/         # ament_cmake  — intent → velocity controller (C++)
│   ├── j10_safety/          # ament_cmake  — safety filter + arbiter (C++)
│   ├── j10_mavlink/         # ament_cmake  — MAVROS bridge + vehicle state (C++)
│   ├── j10_vla/             # ament_python — VLA inference + backend plugins (Python)
│   ├── j10_mission/         # ament_python — mission state machine (Python)
│   ├── j10_telemetry/       # ament_python — latency monitor, diagnostics, recording
│   ├── j10_teleop/          # ament_cmake  — joystick override / deadman (C++)
│   └── j10_sim/             # ament_cmake  — Gazebo worlds, models, SITL launch
├── config/                  # shared parameter YAMLs
├── docs/
└── scripts/                 # SITL bootstrap, latency harness, dev tooling
```

### Boundary rules

These four rules are what keep the architecture from degrading as the code grows:

1. **`j10_interfaces` depends on nothing but `std_msgs` / `geometry_msgs`.** Everything else
   depends on it. The message contract stays stable while node internals churn.
2. **No node subscribes to a `/mavros/*` topic except `j10_mavlink`.** All vehicle state
   reaches the system through `/j10/vehicle/state`. This makes a PX4 migration — or a MAVROS
   version bump — a one-package change instead of a rewrite.
3. **`j10_safety` depends only on `j10_interfaces` + `geometry_msgs`.** It must be
   unit-testable with no simulator, no MAVROS, no GPU. If the safety filter can't be tested
   from a plain gtest binary in under a second, it is built wrong.
4. **`j10_sim` is never a runtime dependency of anything.** It exists only for SITL.

---

## 4. Node architecture

| # | Node | Package | Lang | Rate | Responsibility |
|---|------|---------|------|------|----------------|
| 1 | `video_receiver_node` | j10_video | C++ | 30 Hz | Owns one GStreamer pipeline (`udpsrc → rtph264depay → avdec_h264 → videoconvert → appsink`). Converts frames to `sensor_msgs/Image`, stamping the header with **capture time reconstructed from the RTP timestamp**, not arrival time. Publishes camera info and a per-frame `LatencyReport`. Declares stream loss after 500 ms without a frame. |
| 2 | `vla_inference_node` | j10_vla | Python | 5–10 Hz | Loads the VLA checkpoint. Subscribes to the latest image (KEEP_LAST(1) — **always drops stale frames, never queues**), the active instruction, and vehicle state. Runs inference off the executor thread. Publishes `NavIntent` with confidence and measured inference latency. Never blocks, never retries in-loop. |
| 3 | `motion_controller_node` | j10_control | C++ | 30 Hz | The fast layer. Holds the active `NavIntent` and converts it to a body-frame velocity with trapezoidal accel/decel shaping, so intent changes don't step the command. **Decays to hover when the intent outlives its `duration` or the VLA goes silent.** |
| 4 | `safety_filter_node` | j10_safety | C++ | 30 Hz | The independent guardian, and the only node with authority to veto. Applies in order: input freshness watchdogs → velocity clamps → accel/jerk limits → altitude floor/ceiling → virtual geofence → LiDAR proximity braking → battery/link failsafe. Publishes the filtered command plus a `SafetyStatus` naming every active limit. **Fails to zero velocity; escalates BRAKE → LAND.** |
| 5 | `mavlink_bridge_node` | j10_mavlink | C++ | 30 Hz | Sole owner of the FC interface. Publishes `mavros_msgs/PositionTarget` on `/mavros/setpoint_raw/local` with `coordinate_frame = FRAME_BODY_NED (8)` and a type_mask ignoring position/acceleration/yaw, leaving vx/vy/vz + yaw_rate. **Streams continuously at 30 Hz even when idle** (zeros) because ArduPilot's guided failsafe trips on setpoint silence. Manages GUIDED entry, arming, and mode-loss detection. |
| 6 | `vehicle_state_node` | j10_mavlink | C++ | 20 Hz | Aggregates `/mavros/state`, `/mavros/local_position/odom`, `/mavros/battery`, `/mavros/rangefinder/rangefinder` into one coherent snapshot. Single source of truth for vehicle state. |
| 7 | `mission_manager_node` | j10_mission | Python | 5 Hz | State machine: `IDLE → PREFLIGHT → ARMED → TAKEOFF → VLA_ACTIVE → HOLD → LAND → DISARM`. Owns the natural-language instruction and gates whether VLA output may reach the controller. **Cannot bypass the safety filter.** |
| 8 | `teleop_override_node` | j10_teleop | C++ | 50 Hz | Joystick with a deadman button. Publishes manual velocity and the E-stop latch. Highest arbitration priority — human input instantly preempts autonomy. |
| 9 | `latency_monitor_node` | j10_telemetry | Python | 1 Hz | Aggregates per-stage `LatencyReport` into p50/p95/p99 and publishes `diagnostic_msgs/DiagnosticArray`. **This is how the 300 ms number gets proven rather than assumed.** |
| 10 | `dataset_recorder_node` | j10_telemetry | Python | — | Synchronized rosbag2 capture of image + intent + state + safety verdict, for VLA fine-tuning and post-flight review. |

### Arbitration

One priority ladder, resolved inside `safety_filter_node`, with no ambiguity:

```
E-STOP  >  manual teleop (deadman held)  >  safety override (brake/land)  >  autonomous
```

### Execution model

`MultiThreadedExecutor` with mutually-exclusive callback groups per node. The video receiver,
motion controller, safety filter, and MAVLink bridge run in **one C++ component container with
`use_intra_process_comms:=true`**, which removes serialization from the image and command path
— the cheapest latency win available. The Python VLA node runs as a separate process
(intra-process comms is C++-only) and receives frames over Cyclone DDS shared memory.

---

## 5. Data flow & topics

```
   Pi Zero 2W ──RTP/H.264/UDP──►  video_receiver_node
                                        │ /j10/camera/image_raw
                        ┌───────────────┴───────────────┐
                        ▼                               ▼
                vla_inference_node              dataset_recorder_node
                        │ /j10/vla/intent  @5-10 Hz
                        ▼
                motion_controller_node
                        │ /j10/cmd_vel_raw  @30 Hz
                        ▼
   /j10/cmd_vel_manual ─►  safety_filter_node  ◄─ /j10/vehicle/state
   /j10/safety/estop  ─►         │
                                 │ /j10/cmd_vel_safe  @30 Hz
                                 ▼
                        mavlink_bridge_node
                                 │ /mavros/setpoint_raw/local
                                 ▼
                        MAVROS ──MAVLink/UDP──► ArduPilot (SITL or FC)
                                 │
                                 └─► vehicle_state_node ─► /j10/vehicle/state
```

### Topic contract

| Topic | Type | Rate | QoS | Publisher → Subscriber |
|-------|------|------|-----|------------------------|
| `/j10/camera/image_raw` | `sensor_msgs/Image` | 30 Hz | BEST_EFFORT, KEEP_LAST(1) | video_receiver → vla, recorder |
| `/j10/camera/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | RELIABLE, TRANSIENT_LOCAL | video_receiver → vla |
| `/j10/video/status` | `j10_interfaces/StreamStatus` | 5 Hz | RELIABLE, KEEP_LAST(5) | video_receiver → safety, mission |
| `/j10/mission/instruction` | `std_msgs/String` | on change | RELIABLE, TRANSIENT_LOCAL | mission → vla |
| `/j10/vla/intent` | `j10_interfaces/NavIntent` | 5–10 Hz | RELIABLE, KEEP_LAST(1) | vla → motion_controller |
| `/j10/cmd_vel_raw` | `geometry_msgs/TwistStamped` | 30 Hz | BEST_EFFORT, KEEP_LAST(1) | motion_controller → safety |
| `/j10/cmd_vel_manual` | `geometry_msgs/TwistStamped` | 50 Hz | BEST_EFFORT, KEEP_LAST(1) | teleop → safety |
| `/j10/safety/estop` | `std_msgs/Bool` | on change | RELIABLE, TRANSIENT_LOCAL | teleop → safety |
| `/j10/cmd_vel_safe` | `geometry_msgs/TwistStamped` | 30 Hz | BEST_EFFORT, KEEP_LAST(1) | safety → bridge |
| `/j10/safety/status` | `j10_interfaces/SafetyStatus` | 30 Hz | RELIABLE, KEEP_LAST(10) | safety → mission, telemetry |
| `/j10/vehicle/state` | `j10_interfaces/VehicleState` | 20 Hz | RELIABLE, KEEP_LAST(5) | vehicle_state → all |
| `/j10/telemetry/latency` | `j10_interfaces/LatencyReport` | per-frame | BEST_EFFORT, KEEP_LAST(10) | all stages → latency_monitor |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 Hz | RELIABLE, KEEP_LAST(10) | latency_monitor → operator |

### Why the QoS is split this way

Every realtime path is `BEST_EFFORT / KEEP_LAST(1)`. On a control loop a *late* frame or
command is worse than a *dropped* one — reliable QoS would retransmit stale data and inflate
latency at exactly the moment the link is already degraded.

Safety status, E-stop, and the mission instruction are `RELIABLE`, and latched
(`TRANSIENT_LOCAL`) where a late-joining node must see current state rather than wait for the
next change. Those must never be silently lost.

---

## 6. Latency budget

Design against this table. `latency_monitor_node` measures against it on every flight.

| Stage | Target | Notes |
|-------|--------|-------|
| Capture + H.264 encode (Pi Zero 2W) | 25–40 ms | Hardware `v4l2h264enc`, zero-latency tune, no B-frames |
| WiFi transport | 5–25 ms | 5 GHz, dedicated AP; the dominant variance source |
| Decode + ROS publish | 10–20 ms | GStreamer appsink, intra-process handoff |
| **VLA inference** | **80–150 ms** | The dominant term — the only one worth optimizing hard |
| Controller + safety filter | 3–8 ms | C++, same container, zero-copy |
| MAVROS → FC → actuator | 10–20 ms | 30 Hz setpoint stream, quantization included |
| **End-to-end** | **133–263 ms** | Headroom to 300 ms is thin — measure, don't assume |

If the VLA lands above ~150 ms the cascade absorbs it: the controller keeps commanding at
30 Hz regardless. What degrades is *reaction time to novel scene content*, not command
continuity — which is the right tradeoff, and precisely why the safety filter must be
independent of the model.

---

## 7. ArduPilot configuration (indoor, GPS-denied)

### SITL — simulated flow and rangefinder

```
AHRS_EKF_TYPE   3      EK3_ENABLE      1      GPS_TYPE        0
EK3_SRC1_POSXY  0      EK3_SRC1_VELXY  5      # 5 = OpticalFlow
EK3_SRC1_POSZ   2      EK3_SRC1_VELZ   0      # 2 = RangeFinder
SIM_FLOW_ENABLE 1      FLOW_TYPE       10     # 10 = SITL flow
RNGFND1_TYPE    100    RNGFND1_ORIENT  25     RNGFND1_MAX_CM  800
```

### Real MTF-01 over MSP

Same `EK3_SRC1_*` block, with `SERIALx_PROTOCOL=32` (MSP), `FLOW_TYPE=6` (MSP),
`RNGFND1_TYPE=32` (MSP), `RNGFND1_ORIENT=25`.

> ⚠️ **Verify these three parameter values against the current ArduPilot MTF-01 wiki page
> before first flight.** MSP sensor type IDs have moved between releases, and a wrong
> `FLOW_TYPE` silently yields no flow fusion rather than an error.

> ⚠️ **Calibrate `FLOW_FXSCALER` / `FLOW_FYSCALER` and `FLOW_ORIENT_YAW` on the real
> airframe.** Uncalibrated optical flow produces a slow position drift that looks exactly
> like a control bug, and will cost days of debugging if skipped.

---

## 8. Testing roadmap

Each phase has a hard exit criterion. Do not begin the next phase until it passes.

### Phase 0 — Foundation *(no flight)*
Workspace skeleton, `j10_interfaces` builds, CI green.
**Exit:** `colcon build` clean from scratch in a fresh Humble container.

### Phase 1 — SITL + MAVROS loopback *(no vision, no VLA)*
ArduCopter SITL + Gazebo + MAVROS connected, running only `mavlink_bridge_node` and
`vehicle_state_node`. Hand-publish `/j10/cmd_vel_safe` from the CLI.
**Exit:** a published `TwistStamped` moves the vehicle in the commanded body-frame direction,
and releasing the command returns it to hover within 1 s.

### Phase 2 — Safety filter in isolation *(no simulator at all)*
Full gtest suite on `safety_filter_node`: clamps, jerk limits, geofence, altitude floor, every
watchdog timeout, arbitration priority, E-stop preemption. Then `launch_testing` fault
injection in SITL — kill the VLA mid-flight, sever the video stream, drop the MAVROS link.
**Exit:** every fault produces a bounded, deliberate hover-or-land. **No fault produces a
runaway command.**

> This is the phase not to rush. It is the only thing standing between a model hallucination
> and a wall.

### Phase 3 — Video pipeline + latency harness
Gazebo camera → GStreamer RTP → `video_receiver_node`, then the same against the real Pi.
**Exit:** measured glass-to-ROS p95 under 100 ms; stream loss detected within 500 ms.

### Phase 4 — Scripted-policy end-to-end *(no real model)*
Stub VLA backend emitting a fixed intent sequence. Full chain live.
**Exit:** the drone flies the scripted pattern in Gazebo with fresh timing at every stage.

> This is the real integration milestone — the architecture is proven here, before the model
> is ever a variable.

### Phase 5 — Real VLA in the loop *(simulation only)*
Swap in the real checkpoint. Profile, quantize, tune input resolution.
**Exit:** end-to-end p95 under 300 ms; a simple indoor navigation task completes in Gazebo.

### Phase 6 — Hardware-in-the-loop *(props OFF)*
Real Pixhawk, Pi, and camera with ArduPilot still in SITL. Real WiFi, real encode latency.
**Exit:** latency budget holds over a 10-minute continuous run with no watchdog trip.

### Phase 7 — Tethered indoor flight
Props on, tether attached, human on the deadman switch, conservative envelope (0.3 m/s,
1.5 m ceiling, 2×2 m geofence). Expand limits only after each envelope flies clean.
**Exit:** repeatable autonomous indoor navigation with zero safety interventions across
5 flights.

---

## 9. Build order

| Step | Packages | Enables |
|------|----------|---------|
| 1 | `j10_interfaces` | The message contract — everything keys off it |
| 2 | `j10_mavlink`, `j10_sim` | Phase 1 |
| 3 | `j10_safety`, `j10_control` | Phase 2 |
| 4 | `j10_video`, `j10_telemetry` | Phase 3 |
| 5 | `j10_vla` (stub), `j10_mission`, `j10_teleop` | Phase 4 |
| 6 | `j10_vla` (real backend) | Phase 5 |

---

## 10. Scope

This document covers the PC-side workspace architecture. It does **not** cover:

- Training or fine-tuning the VLA model
- The Pi-side streaming service (a standalone GStreamer unit, not a ROS 2 node)
- ArduPilot firmware modifications

Those are separate efforts that plug into the contracts defined above.
