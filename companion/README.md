# J10 PT1 — MAVLink Bridge (Pi Zero 2W ⇄ CUAV V7 Nano)

Standalone companion-computer microservice. It owns the **Serial/UART link to the flight
controller** and nothing else: no perception, no planning, no vision. It receives
`(vx, vy, vz, yaw_rate)` from an independent CV process over a shared-memory adapter and
streams it to the FC as MAVLink velocity setpoints, at a fixed rate, forever — falling back
to zero-velocity hover the instant that input goes missing, stale, or invalid.

This is the onboard PT1 track: Pi Zero 2W → Serial UART → CUAV V7 Nano, CV-driven, no VLA,
no ground-station PC in the loop. It intentionally does **not** live under `src/` — that
ROS 2 workspace is the separate PC-side offboard-control architecture described in
[`../docs/ARCHITECTURE.md`](../docs/ARCHITECTURE.md), which already carves out "the Pi-side
streaming service" as explicitly out of its scope. `companion/` is where Pi-side, non-ROS
software lives.

```
┌─────────────────────────────── Raspberry Pi Zero 2W ───────────────────────────────┐
│                                                                                      │
│   cv_node (separate package, not yet written)                                      │
│       │ writes (vx, vy, vz, yaw_rate) @ its own rate, via j10_shm_protocol         │
│       ▼                                                                            │
│   /dev/shm/j10_cv_cmd  ◄── j10_shm_protocol: seqlock, 36 bytes, stdlib-only ──►    │
│       ▲                                                                            │
│       │ reads latest command every setpoint tick, via j10_shm_protocol            │
│   mavlink_bridge (this package)                                                   │
│       │ SET_POSITION_TARGET_LOCAL_NED @ 20 Hz (zeros on any failsafe)             │
│       ▼                                                                            │
│   /dev/serial0 (UART) ──────────────────────────────────────────┐                  │
└───────────────────────────────────────────────────────────────┼──────────────────┘
                                                                    ▼
                                                       CUAV V7 Nano (ArduPilot)
```

`j10_shm_protocol` is its own installable package ([`j10_shm_protocol/`](j10_shm_protocol/)),
not part of `mavlink_bridge` — see "Shared-memory adapter" below for why.

## Why Python, not C++

The task budget here is one ~36-byte struct read plus one ~50-byte MAVLink message encode
and one `pyserial` write, at 20 Hz. That's microseconds of work on a Pi Zero 2W's
Cortex-A53; `pymavlink` (the reference MAVLink implementation, used by ArduPilot's own
tooling) already generates the message pack/unpack code in C via `pymavlink`'s
mavgen-generated `MAVLink_message` classes, so the "hot path" isn't actually pure-Python
overhead. C++ would buy negligible latency here and cost real development time versus a
project whose critical unknowns (CV inference, EKF3 tuning, indoor navigation) are
elsewhere. Reassess if profiling ever shows this loop is the bottleneck — it won't be before
the CV node is.

## Layout

```
companion/
├── j10_shm_protocol/                     # standalone package: the shared-memory contract
│   ├── pyproject.toml                    # installable as `j10-shm-protocol`
│   ├── j10_shm_protocol/__init__.py      # CVCommand, CVCommandReader, CVCommandWriter
│   └── tests/                            # pytest, no hardware required
├── mavlink_bridge/                       # depends on j10_shm_protocol, not the other way
│   ├── config.py                         # BridgeConfig — env-var configurable
│   ├── bridge.py                         # MavlinkBridge — the actual service
│   ├── cv_stub.py                        # bench-test stand-in for the real CV node
│   └── __main__.py                       # `python -m mavlink_bridge`
├── pyproject.toml                        # installable as `j10-mavlink-bridge`
├── requirements.txt                      # plain pip install alternative
├── tests/                                # pytest, no hardware required
└── systemd/j10-mavlink-bridge.service
```

(`cv_node/` — the real CV package — lands here too, as a third sibling depending on
`j10_shm_protocol` the same way `mavlink_bridge` does; see the bottom of this README.)

## Running it

```bash
cd companion
python3 -m venv .venv && source .venv/bin/activate

# j10_shm_protocol first — mavlink_bridge depends on it and it isn't on PyPI, so it has to
# already be installed (editable is fine) before `pip install -e .` below can resolve it.
pip install -e ./j10_shm_protocol
pip install -e ".[dev]"

pytest -v j10_shm_protocol/tests tests    # 18 tests total, no serial port or FC needed

# terminal 1 — stand in for the CV node: constant 0.2 m/s forward
j10-cv-stub --vx 0.2

# terminal 2 — the bridge itself (needs a real or SITL FC on the configured serial port)
J10_BRIDGE_SERIAL_PORT=/dev/serial0 j10-mavlink-bridge
```

Use the `pytest`, `j10-cv-stub`, and `j10-mavlink-bridge` commands above rather than
`python -m pytest` / `python -m mavlink_bridge...` while your shell is inside `companion/`.
`-m` prepends the current directory to `sys.path`, and `companion/j10_shm_protocol/` (that
package's *project root*, not the package itself — no `__init__.py` at that level) would
then shadow the real, pip-installed `j10_shm_protocol` with an empty namespace package,
producing a confusing `ImportError`/`ModuleNotFoundError`. The installed console scripts
sidestep this the same way the `pytest` command does; `python -m ...` still works fine from
any directory that doesn't contain a folder literally named `j10_shm_protocol` (e.g. the
repo root). See the `[tool.pytest.ini_options]` comment in `pyproject.toml` for the
mechanics if you hit this while poking around.

To watch the failsafe engage without touching hardware: `Ctrl-C` the `cv_stub`, or run it
with `--duration 5` and watch the bridge's log switch to `failsafe hover engaged: CV
command stale (...)` within `cv_stale_timeout_s` (default 250 ms) of it stopping.

On the Pi, install the systemd unit for a supervised, auto-restarting deployment:

```bash
sudo cp systemd/j10-mavlink-bridge.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now j10-mavlink-bridge
journalctl -u j10-mavlink-bridge -f
```

## Wiring the Pi to the CUAV V7 Nano

1. Connect a free UART on the V7 Nano (e.g. `TELEM2`/`UART4`, per the CUAV V7 Nano manual's
   pinout — confirm the port before wiring) to the Pi's primary UART: FC-TX → Pi RXD (GPIO
   15 / pin 10), FC-RX → Pi TXD (GPIO 14 / pin 8), GND → GND. **Do not** connect the FC's
   5V to the Pi — the BOM's dedicated 5V/3A BEC powers the Pi.
2. On the Pi (Raspberry Pi OS / Bookworm), free the primary UART from the Linux serial
   console and disable the Bluetooth UART hand-off so `/dev/serial0` maps to the PL011 UART:
   `sudo raspi-config` → *Interface Options → Serial Port* → login shell over serial: **No**,
   serial hardware enabled: **Yes**; or directly in `/boot/firmware/config.txt`:
   ```
   enable_uart=1
   dtoverlay=disable-bt
   ```
   Reboot after either change.
3. On the FC side, set the matching `SERIALx_PROTOCOL=2` (MAVLink2) and `SERIALx_BAUD` to
   match `J10_BRIDGE_BAUD` (default 921600 → `SERIALx_BAUD=921`) for whichever `SERIALx`
   maps to the physical port used in step 1.

## MAVLink reference

Everything below is what `bridge.py` actually sends/relies on. Verify version-sensitive
values (frame IDs, MSP/rangefinder types) against the current ArduPilot release before
first flight — MAVLink enum *numeric values* are stable/append-only by spec, but which
frames and modes a given ArduPilot version accepts has moved before (see
[`../docs/ARCHITECTURE.md`](../docs/ARCHITECTURE.md) §7 for a prior example with the
MTF-01).

| Concept | Value used here | Source |
|---|---|---|
| Companion heartbeat | `HEARTBEAT` @ 1 Hz, `type=MAV_TYPE_ONBOARD_CONTROLLER` (18), `autopilot=MAV_AUTOPILOT_INVALID` (8) | [MAVLink HEARTBEAT](https://mavlink.io/en/messages/common.html#HEARTBEAT), [MAV_TYPE](https://mavlink.io/en/messages/minimal.html#MAV_TYPE), [MAV_AUTOPILOT](https://mavlink.io/en/messages/minimal.html#MAV_AUTOPILOT) |
| Component ID | `MAV_COMP_ID_ONBOARD_COMPUTER` = 191 | [MAV_COMPONENT](https://mavlink.io/en/messages/common.html#MAV_COMPONENT) |
| Setpoint message | `SET_POSITION_TARGET_LOCAL_NED` (msg id 84) | [MAVLink SET_POSITION_TARGET_LOCAL_NED](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED) |
| Coordinate frame | `MAV_FRAME_BODY_NED` = 8 (matches the frame already chosen for the ROS 2 bridge in `docs/ARCHITECTURE.md`) | [MAV_FRAME](https://mavlink.io/en/messages/common.html#MAV_FRAME) |
| type_mask | `0b0000010111000111` = `1479` = `0x5C7` — see derivation below | [POSITION_TARGET_TYPEMASK](https://mavlink.io/en/messages/common.html#POSITION_TARGET_TYPEMASK), [ArduPilot Copter Commands in Guided Mode](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html) (SET_POSITION_TARGET_LOCAL_NED section — lists the same "Velocity + yaw rate" mask as a worked example) |
| Guided-mode setpoint watchdog | ArduPilot stops trusting an idle offboard stream after a few seconds of silence, which is why the bridge streams continuously (including zeros) rather than only when it has something to say | [ArduPilot Guided mode](https://ardupilot.org/copter/docs/ac2_guidedmode.html), [ArduPilot GCS Failsafe](https://ardupilot.org/copter/docs/gcs-failsafe.html) |
| Reference implementation | `pymavlink` (ArduPilot's own MAVLink Python bindings, used by `mavproxy`, `DroneKit`, and ArduPilot's own test suite) | [ArduPilot/pymavlink](https://github.com/ArduPilot/pymavlink) |

### `type_mask` derivation

`POSITION_TARGET_TYPEMASK` is 12 independent "ignore this field" bits, each a single power
of two, in this fixed order:

| bit | value | field |
|---|---|---|
| 0 | 1 | X (position) |
| 1 | 2 | Y (position) |
| 2 | 4 | Z (position) |
| 3 | 8 | VX (velocity) |
| 4 | 16 | VY (velocity) |
| 5 | 32 | VZ (velocity) |
| 6 | 64 | AFX (acceleration) |
| 7 | 128 | AFY (acceleration) |
| 8 | 256 | AFZ (acceleration) |
| 9 | 512 | FORCE_SET (treat accel field as force instead) |
| 10 | 1024 | YAW (absolute) |
| 11 | 2048 | YAW_RATE |

We want the FC to use velocity (vx, vy, vz) and yaw_rate, and ignore everything else —
i.e. set every "ignore" bit *except* VX/VY/VZ_IGNORE and YAW_RATE_IGNORE:

```
mask = X_IGNORE | Y_IGNORE | Z_IGNORE | AFX_IGNORE | AFY_IGNORE | AFZ_IGNORE | YAW_IGNORE
     =    1     +    2     +    4     +     64     +    128     +    256     +   1024
     = 1479  (0x5C7, 0b0000010111000111)
```

This is checked in `tests/test_setpoint_encoding.py` directly against the bit table above,
so a future edit that breaks the mask fails CI rather than fails in the air.

### Message construction (`bridge.py::_send_setpoint`)

```python
master.mav.set_position_target_local_ned_send(
    time_boot_ms,                          # ms since bridge start, wraps at 2^32
    target_system, target_component,       # learned from the FC's own HEARTBEAT
    coordinate_frame,                      # MAV_FRAME_BODY_NED (8)
    type_mask,                             # 1479 — velocity + yaw_rate only
    0.0, 0.0, 0.0,                         # x, y, z            — ignored
    vx, vy, vz,                            # commanded velocity, m/s, body frame
    0.0, 0.0, 0.0,                         # afx, afy, afz      — ignored
    0.0,                                   # yaw                — ignored
    yaw_rate,                              # commanded yaw rate, rad/s
)
```

## Shared-memory adapter (`j10_shm_protocol` package)

POSIX shared memory via `multiprocessing.shared_memory` — one 36-byte segment,
`/dev/shm/j10_cv_cmd` by default, no serialization and no broker. It lives in
[`j10_shm_protocol/`](j10_shm_protocol/) as its **own standalone, dependency-free package**
rather than inside `mavlink_bridge`, precisely so the CV node doesn't have to import the
bridge's package (or vice versa) just to speak the wire format both of them share — see
that package's README for why, and its own test suite for the contract in isolation.

Layout and the seqlock concurrency scheme are documented in that package's module
docstring; short version: the writer (CV node) bumps a sequence counter odd-then-even
around every write, the reader (this bridge) retries a bounded number of times if it
catches a write in progress, and any failure to get a clean read — including "the segment
doesn't exist yet" — is treated exactly like a stale command by the bridge's failsafe
logic. Nothing here blocks either process.

`mavlink_bridge/cv_stub.py` implements the writer half so you can bench-test the bridge
(including the failsafe path) before the real CV node exists. A real CV node just needs:

```python
from j10_shm_protocol import CVCommandWriter
writer = CVCommandWriter(name="j10_cv_cmd")
...
writer.write(vx=0.2, vy=0.0, vz=0.0, yaw_rate=0.1, valid=True)
```

> Note: if a writer process exits without calling `.unlink()` (e.g. it's killed rather than
> shut down cleanly, or is a supervised service like this bridge expects it to be), Linux's
> shared-memory resource tracker prints a `resource_tracker: leaked shared_memory objects`
> warning to stderr. That's expected here, not a bug — the whole point of the microservice
> split is that the CV node and the bridge restart independently, and `CVCommandWriter`
> reattaches to a segment left behind by a previous run instead of crashing (see
> `test_writer_survives_stale_segment_left_by_a_crashed_writer`).

## Failsafe behavior (requirement #4)

`MavlinkBridge._compute_setpoint()` returns zero velocity, unconditionally, whenever any of
these hold — checked on every single setpoint tick, not just on state transitions:

1. No CV command has ever been written (`CVCommandReader.read()` → `None`).
2. The CV node explicitly marked its own command `valid=False`.
3. The most recent command is older than `cv_stale_timeout_s` (default 250 ms) — this is
   what catches a CV-node crash, hang, or a shared-memory read that keeps racing torn
   writes.
4. The flight controller's own `HEARTBEAT` hasn't been seen in `fc_link_timeout_s` (default
   3 s) — this catches the serial link itself dying, independent of the CV node.

Everything else — a fresh, valid, in-range command — is clamped to the configured indoor
velocity envelope (`max_horizontal_speed_mps`, `max_vertical_speed_mps`,
`max_yaw_rate_rps`; conservative bench-test defaults per
[`../docs/ARCHITECTURE.md`](../docs/ARCHITECTURE.md) §Phase 6/7) and sent as-is. The clamp
applies regardless of what the CV node asks for — see
`test_velocity_envelope_clamps_regardless_of_what_cv_node_requests`.

On top of the per-tick failsafe, `MavlinkBridge.run()` sends a short burst of zero-velocity
setpoints on any shutdown path (`SIGINT`/`SIGTERM`, or falling out of the main loop) before
closing the serial link, so a service restart never leaves "whatever the last real command
was" as the last thing the FC heard before the setpoint stream goes silent.

All of this is covered by `tests/test_bridge_failsafe.py` against a mocked FC connection —
no serial port, no SITL, no props — per the project's standing rule
([`../README.md`](../README.md) "Safety") that safety-relevant logic must be testable
without hardware.

## Safety posture

This bridge **never arms the vehicle and never changes flight mode by default**
(`auto_request_guided_mode: bool = False` in `config.py`). The BOM's ELRS receiver +
handheld transmitter is the intended arming/mode authority for PT1 bench and tethered
testing, consistent with the human-priority arbitration already established for the
project (`../docs/ARCHITECTURE.md` §4 "Arbitration": manual input outranks autonomy).
Flip `auto_request_guided_mode` only once that decision has been made deliberately, not as
a default.

Props-off / tethered testing discipline from the top-level README and
`docs/ARCHITECTURE.md` §8 (Phase 6/7) applies to this bridge exactly as it does to the
ROS 2 stack: bring up and validate the failsafe path with propellers removed first.

## Future: where `cv_node` lands

The real CV package will live at `companion/cv_node/`, as a third sibling next to
`j10_shm_protocol/` and `mavlink_bridge/` — same pattern: its own `pyproject.toml`, its own
tests, depending on `j10_shm_protocol` (now that it's already split out) to write commands
instead of reaching into `mavlink_bridge`'s internals. `mavlink_bridge/cv_stub.py` is what
it replaces, not something it extends.
