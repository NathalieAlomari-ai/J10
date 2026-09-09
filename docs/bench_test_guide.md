# PT1 Bench Test Guide

The step-by-step guide for bringing up the onboard PT1 stack — Pi Camera Module 3 →
`cv_node` → `j10_shm_protocol` → `mavlink_bridge` → CUAV V7 Nano — on the bench. Written
for Mahmoud to run this end to end without needing to read the three `companion/`
packages' own READMEs first, though those are the deeper reference this guide
summarizes: [`companion/README.md`](../companion/README.md) and
[`companion/cv_node/README.md`](../companion/cv_node/README.md).

This is PT1, not the PC-side VLA architecture in [`ARCHITECTURE.md`](ARCHITECTURE.md) —
no ground-station PC, no ROS 2, CV-driven navigation running entirely on the Pi. See that
doc's top-level scope note if it's unclear which track a given file belongs to.

---

## ⚠️ Safety — read this before touching anything

1. **Propellers OFF for the entire bench test.** Every step in this guide — including
   arming, if you get that far — is meant to be done with props physically removed. This
   mirrors the project's standing rule
   ([top-level README](../README.md) "Safety", `ARCHITECTURE.md` Phase 6/7): bring up and
   validate the failsafe path with props off before they ever go back on.
2. **This software never arms the vehicle and never changes flight mode.**
   `mavlink_bridge` streams velocity setpoints unconditionally, but ArduPilot ignores
   offboard setpoints unless the vehicle is armed and in GUIDED — and arming/mode changes
   are deliberately left to the human on the ELRS transmitter
   (`companion/README.md` "Safety posture"). If you arm during this bench test, you are
   the one doing it, on purpose, watching a deadman switch.
3. **The failsafe test in Step 6 is not optional.** Confirming the bridge drops to
   zero-velocity hover when the CV node goes silent is the one thing that most needs to be
   true before this ever flies. Do it every time you bring the stack up on new hardware or
   after any code change.

---

## 1. Hardware prerequisites

### What this bench test needs (from the BOM)

| Component | Role in this test |
|---|---|
| CUAV V7 Nano | Flight controller — receives the MAVLink velocity setpoints |
| Raspberry Pi Zero 2W | Companion computer — runs both `cv_node` and `mavlink_bridge` |
| Raspberry Pi Camera Module 3 | `cv_node`'s only sensor input |
| 5V/3A BEC | Powers the Pi. **Do not power the Pi from the FC's 5V rail** — separate supply, per `companion/README.md` "Wiring" |
| ELRS receiver + handheld transmitter | Arming/mode authority — the human's, not the software's (see Safety §2 above) |
| Micoair MTF-01 (optical flow + LiDAR), TFmini-S | Not exercised by this specific bench test (no arming, no GUIDED flight), but must be wired and configured per `ARCHITECTURE.md` §7 before any step past this guide — indoor EKF3 has no GPS to fall back on |

Airframe, motors, ESCs, props: physically present is fine, but **props stay off** — see
Safety §1. Nothing in this guide needs the vehicle to actually be capable of flight.

### Wiring

Full pin-level detail is in `companion/README.md` "Wiring the Pi to the CUAV V7 Nano" —
summarized:

- FC UART (e.g. `TELEM2`/`UART4` — confirm against the V7 Nano manual) → Pi GPIO: FC-TX →
  Pi RXD (GPIO 15 / pin 10), FC-RX → Pi TXD (GPIO 14 / pin 8), GND → GND.
- Pi: `raspi-config` → *Interface Options → Serial Port* → login shell over serial **No**,
  serial hardware **Yes** (or `enable_uart=1` / `dtoverlay=disable-bt` in
  `/boot/firmware/config.txt`), then reboot — this is what makes `/dev/serial0` the real
  UART instead of the login console.
- FC: `SERIALx_PROTOCOL=2` (MAVLink2), `SERIALx_BAUD=921` on whichever `SERIALx` maps to
  the port wired above (matches the bridge's default `921600` baud).
- Camera Module 3: connected to the Pi's CSI port, enabled and testable independently of
  this stack first (`libcamera-hello` or equivalent) — if the camera doesn't work outside
  this software, it won't work inside it either.

---

## 2. Architecture / data flow

```
┌─────────────────────────────── Raspberry Pi Zero 2W ───────────────────────────────┐
│                                                                                      │
│   cv_node: Pi Camera Module 3 -> grid-based obstacle avoidance                     │
│       │ writes (vx, vy, vz, yaw_rate) @ ~8 Hz, via j10_shm_protocol                │
│       ▼                                                                            │
│   /dev/shm/j10_cv_cmd  ◄── j10_shm_protocol: seqlock, 36 bytes, stdlib-only ──►    │
│       ▲                                                                            │
│       │ reads latest command every setpoint tick, via j10_shm_protocol            │
│   mavlink_bridge                                                                   │
│       │ SET_POSITION_TARGET_LOCAL_NED @ 20 Hz (zeros on any failsafe)             │
│       ▼                                                                            │
│   /dev/serial0 (UART) ──────────────────────────────────────────┐                  │
└───────────────────────────────────────────────────────────────┼──────────────────┘
                                                                    ▼
                                                       CUAV V7 Nano (ArduPilot)
```

Three independent processes, one contract between them:

| Stage | Package | Rate | Job |
|---|---|---|---|
| Vision | `cv_node` | ~8 Hz | Camera frame → Left/Center/Right edge-density scoring → `(vx, vy, vz, yaw_rate)`. No MAVLink, no Serial. |
| Wire contract | `j10_shm_protocol` | — | 36-byte POSIX shared-memory segment, seqlock (no mutex), stdlib-only. Neither service depends on the other's package — only on this one. |
| Control | `mavlink_bridge` | 20 Hz | Reads the latest command, clamps it to a conservative velocity envelope, streams it as MAVLink. **Fails to zero-velocity hover** the instant its input is missing, stale, invalid, or the FC link itself goes quiet. |

Why two different rates: this mirrors the two-rate cascade already established for the
PC-side architecture (`ARCHITECTURE.md` §2) — the slow layer (`cv_node`) decides
*direction*, the fast layer (`mavlink_bridge`) guarantees the FC always has a fresh,
bounded command regardless of how long a vision tick takes, including if `cv_node` stalls
or is killed outright. Neither process needs to know the other exists beyond the shared
memory segment; that's the whole point of the split.

---

## 3. Software setup

```bash
cd ~/J10   # or wherever this checkout lives on the Pi
git pull --ff-only origin main

sudo apt update
sudo apt install -y python3-opencv python3-picamera2 python3-numpy

python3 -m venv --system-site-packages companion/.venv
source companion/.venv/bin/activate

# Order matters: j10_shm_protocol first (nothing depends on it; both other packages
# depend on it, and it isn't on PyPI, so it has to already be installed before pip can
# resolve their dependency on it).
pip install -e ./companion/j10_shm_protocol
pip install -e "./companion[dev]"
pip install -e ./companion/cv_node --no-deps   # --no-deps: opencv/numpy/picamera2 come
                                                 # from apt above, not pip -- see
                                                 # companion/cv_node/README.md "Installing
                                                 # on the Pi" for why (a Pi Zero 2W's
                                                 # 512MB is not enough for pip to build
                                                 # OpenCV from source if piwheels doesn't
                                                 # have your exact combination prebuilt).

pytest -v companion/j10_shm_protocol/tests companion/tests companion/cv_node/tests
```

That last line is a sanity gate, not optional — 45 tests, none of which touch a camera,
a serial port, or any hardware. If any fail, stop and fix the install before continuing;
everything past this point assumes the software itself is sound and is only testing the
hardware integration.

> **Shortcut:** if `test1.sh` is present at the repo root, it automates everything in
> this section (pull, venv, install order, test run) and ends by printing the exact
> commands from Step 5 below. Run `./test1.sh` from the repo root instead of the block
> above if you have it. *At the time this guide was written, `test1.sh` was still on an
> open pull request, not yet on `main` — if `git pull` above doesn't produce a
> `test1.sh` in the repo root, it hasn't merged yet; use the manual steps above.*

> **Pitfall to know about, not hit:** run `pytest`/`j10-cv-node`/`j10-mavlink-bridge` as
> shown (the installed commands), not `python -m pytest` / `python -m cv_node` /
> `python -m mavlink_bridge`, while your shell is inside `companion/`. `-m` prepends the
> current directory to Python's import path, and `companion/j10_shm_protocol/` (that
> package's *project folder*, not the package itself) then shadows the real, installed
> `j10_shm_protocol` with an empty stand-in — you'll get a confusing `ImportError` that
> has nothing to do with your actual code. Full mechanics in
> `companion/README.md` "Running it" if you're curious why.

---

## 4. Bring up the two services

Two terminals, both `cd ~/J10/companion && source .venv/bin/activate` first.

### Terminal 1 — the CV node

```bash
J10_CV_CAMERA_BACKEND=picamera2 j10-cv-node
```

Expect a startup log line (`J10 CV node starting: CVNodeConfig(...)`) and then silence at
`INFO` level — it's running fine, just not logging every tick unless you raise the level
(`J10_CV_LOG_LEVEL=DEBUG`) to see the per-frame zone densities and chosen direction.

### Terminal 2 — the MAVLink bridge

```bash
J10_BRIDGE_SERIAL_PORT=/dev/serial0 j10-mavlink-bridge
```

---

## 5. What "it's working" looks like

Watch Terminal 2's log, in order:

1. `opening serial link to FC on /dev/serial0 @ 921600 baud`
2. **`FC HEARTBEAT received: system=... component=... type=... autopilot=...`** — the
   serial link and wiring are good. If this hangs instead, it's wiring or baud, not the
   software; see `companion/README.md`'s `connect()` error message and the wiring section
   above before looking anywhere else.
3. Initially: `failsafe hover engaged: no CV command (node down?)` — expected and correct
   the instant the bridge starts, before `cv_node`'s first write lands.
4. Within about a second of both being up: **`failsafe cleared, resuming CV-commanded
   velocity`** — `cv_node`'s commands are actually reaching the bridge through shared
   memory. This is the line that confirms the whole chain, camera to bridge, is live.

If you never see line 4, the shared-memory handoff is the thing to debug — confirm both
processes were started with the same `shm_name` (default `j10_cv_cmd` on both sides,
unless you overrode `J10_BRIDGE_SHM_NAME` / `J10_CV_SHM_NAME`).

---

## 6. Test the failsafe — do this before anything flies

The one thing that must be true before this stack is trusted near a real airframe:

1. With both terminals running and Terminal 2 showing `failsafe cleared, ...`,
   **`Ctrl-C` Terminal 1** (the CV node).
2. Within `cv_stale_timeout_s` of that (default **250 ms**), Terminal 2 must log:
   `failsafe hover engaged: CV command stale (... ms > 250 ms)`.
3. Restart Terminal 1. Confirm Terminal 2 logs `failsafe cleared, ...` again on its own,
   with no restart needed on the bridge side.

If step 2 doesn't happen — if the bridge keeps streaming the CV node's last command
instead of dropping to zero — **stop here and do not proceed to arming under any
circumstances.** That's exactly the failure mode
(`docs/ARCHITECTURE.md` "The fast layer's default output is zero velocity (hover), not
'last command.' Repeating a stale command on loss of input is how offboard systems fly
into walls.") this whole split exists to prevent, and it means something in the install
or the wiring is wrong, not just untested.

Two more failure modes worth forcing while you're set up, from
`companion/README.md`/`companion/cv_node/README.md`:

- Unplug the FC's TX wire — Terminal 2 should log `failsafe hover engaged: FC link
  timeout` within `fc_link_timeout_s` (default 3s), independent of whether `cv_node` is
  still running.
- Cover the camera lens or point it at a blank wall close-up — `cv_node` should log (at
  `DEBUG`) `blocked=True`, and Terminal 2's commanded velocity should go to zero even
  though the CV node is still alive and writing `valid=True` commands. This is the node
  choosing to stop, not the bridge's failsafe — see `cv_node/README.md`'s "Known
  limitation" section for what this heuristic can and can't be trusted to catch.

---

## 7. Troubleshooting

| Symptom | Likely cause |
|---|---|
| `TimeoutError: no HEARTBEAT from flight controller` | Wiring (TX/RX swapped is the classic one), wrong `SERIALx`, or `SERIALx_BAUD` mismatch. Confirm the FC is even powered and its `SERIALx_PROTOCOL=2`. |
| `ImportError: cannot import name '...' from 'j10_shm_protocol' (unknown location)` | The `python -m` / cwd pitfall in §3 — use the installed commands instead, or run from a directory that isn't `companion/`. |
| `cv_node` fails to start: `failed to initialize CV node (camera backend 'picamera2')` | `picamera2` not actually importable — confirm `python3 -c "import picamera2"` works against the **system** Python (not inside the venv) and that the venv was created with `--system-site-packages`. |
| Bridge never logs `failsafe cleared` | See §5 above — confirm matching `shm_name` on both sides first. |
| `RuntimeError: could not open camera device index 0` (only relevant if using `J10_CV_CAMERA_BACKEND=opencv` to bench-test off-Pi) | No USB webcam at that index — this backend is for dev-machine testing, not the Pi bench test; use `picamera2` on the Pi. |
| Tests pass but the real hardware doesn't | The 45-test suite deliberately covers none of the hardware path (camera, serial, FC) — that's what §5 and §6 are for. A green test suite means the *logic* is sound, not that the wiring is. |

---

## References

- [`companion/README.md`](../companion/README.md) — full wiring detail, MAVLink message/
  bitmask reference with citations, failsafe implementation detail.
- [`companion/cv_node/README.md`](../companion/cv_node/README.md) — the obstacle-avoidance
  heuristic, its known limitation, tuning guidance.
- [`companion/j10_shm_protocol/README.md`](../companion/j10_shm_protocol/README.md) — the
  shared-memory wire contract.
- [`ARCHITECTURE.md`](ARCHITECTURE.md) — the separate PC-side VLA track this project also
  contains; §7 has the EKF3/MTF-01 indoor GPS-denied configuration this bench test's
  hardware assumes is already set up.
- [`../README.md`](../README.md) — top-level project scope and the two non-negotiable
  safety rules referenced throughout this guide.
