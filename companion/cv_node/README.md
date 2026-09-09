# J10 PT1 — CV Node (Pi Camera Module 3)

The vision half of PT1: turns a camera frame into a `(vx, vy, vz, yaw_rate)` command and
writes it to shared memory for `mavlink_bridge` to pick up and stream to the CUAV V7 Nano.
Grid-based, non-ML, deliberately lightweight — no model to load, no GPU, no frame history
beyond one exponential-moving-average filter on the output.

This node owns exactly one job: camera → velocity command. It knows nothing about
MAVLink, Serial, or the flight controller — see [`../README.md`](../README.md) for how
the three `companion/` packages fit together, and
[`../j10_shm_protocol/README.md`](../j10_shm_protocol/README.md) for the wire contract
this node writes to.

## The heuristic

Every tick:

1. Grab a frame, convert to grayscale.
2. Crop to a region of interest (cuts out the ceiling near the top and the floor
   immediately under the drone near the bottom — neither says anything useful about
   "what's ahead").
3. Blur lightly, run Canny edge detection.
4. Split into three equal-width columns — Left / Center / Right.
5. Score each column by edge-pixel density (fraction of pixels Canny marked as an edge).
6. Steer toward whichever side reads clearest **only if** it beats Center by more than a
   hysteresis margin; otherwise go straight. If every zone reads as cluttered, stop and
   hold rather than push forward into the "least bad" option.
7. Smooth the output (EMA) so a single noisy frame doesn't produce a visible jerk.

All of this lives in `cv_node/obstacle_avoidance.py` as two pure functions
(`compute_zone_edge_densities`, `decide`) with no camera, no shared memory, and no
threading involved — see that module's docstring and `tests/test_obstacle_avoidance.py`
for the full decision table (clear center, clear side, everything blocked, razor-thin
margins, ties).

### Known limitation — read this before trusting it near anything solid

Edge density is a proxy for "close/cluttered," not a distance measurement. A **close,
blank, low-texture surface** (a plain wall dead ahead) can score as *clear* because it has
few edges, while a **distant but highly textured** floor or patterned carpet can score as
*cluttered*. This is a real, known failure mode of the heuristic, not an edge case to
patch away — it's why the BOM carries a TFmini-S and an MTF-01, and why the final safety
envelope and failsafe live in `mavlink_bridge`, independent of this node, matching the
project's standing rule that safety enforcement doesn't depend on the model/heuristic
producing the command (see the top-level README "Safety" and
`docs/ARCHITECTURE.md` §"Safety"). Treat this node's output as *a* signal, not *the*
signal, until it's fused with range data.

The requirements this node was built against also mention optical flow (time-to-contact
via flow magnitude) as an alternative — a more physically-grounded signal for exactly this
problem, but one that needs frame-to-frame feature tracking, which costs meaningfully more
CPU on a Pi Zero 2W. Left out of v1 deliberately, not by oversight: `compute_zone_edge_densities`
is the only function a `compute_zone_flow_magnitudes` sibling would need to sit next to,
without touching `decide()` or anything upstream of it, if/when that's worth the CPU
budget.

## Layout

```
cv_node/
├── pyproject.toml              # installable as `j10-cv-node`
├── requirements.txt            # plain pip install alternative (dev machine / CI)
├── cv_node/
│   ├── config.py                # CVNodeConfig — env-var configurable
│   ├── obstacle_avoidance.py    # the heuristic: pure functions, no I/O
│   ├── smoothing.py             # EmaSmoother — output smoothing, its own tiny class
│   ├── camera.py                # FrameSource: PiCamera2Source / OpenCVCameraSource / SyntheticFrameSource
│   ├── node.py                  # CVNode — wires camera -> obstacle_avoidance -> j10_shm_protocol
│   └── __main__.py              # `j10-cv-node` console script
└── tests/                       # pytest — no camera or hardware required
```

## Installing on the Pi

`opencv-python-headless` and `picamera2` are both a bad time to `pip install` on a Pi Zero
2W: OpenCV's wheel is a slow, RAM-heavy build if piwheels doesn't have a prebuilt one for
your exact OS/Python combination (512 MB of RAM is not much headroom for a C++ compile),
and `picamera2` isn't pip-installable at all in the usual sense — it's tied to
`libcamera`'s system bindings and ships as the `python3-picamera2` **apt** package on
Raspberry Pi OS. Use apt for both, and let a `--system-site-packages` venv see them:

```bash
sudo apt update
sudo apt install -y python3-opencv python3-picamera2 python3-numpy

python3 -m venv --system-site-packages companion/cv_node/.venv
source companion/cv_node/.venv/bin/activate

# j10_shm_protocol still comes from this checkout, same as mavlink_bridge:
pip install -e ../j10_shm_protocol
# --no-deps: opencv/numpy/picamera2 are already on the system site-packages path above;
# letting pip resolve them normally would try to build/download them anyway.
pip install -e . --no-deps
```

On a dev machine (no Pi, bench-testing the vision logic with a laptop webcam instead), a
plain install is fine — real wheels exist for desktop platforms:

```bash
cd companion/cv_node
python3 -m venv .venv && source .venv/bin/activate
pip install -e ../j10_shm_protocol
pip install -e ".[dev]"
```

## Running it

```bash
pytest -v tests                          # no camera, no hardware required

# bench-test against a laptop webcam before touching the Pi Camera Module 3:
J10_CV_CAMERA_BACKEND=opencv j10-cv-node

# the real thing, on the Pi:
J10_CV_CAMERA_BACKEND=picamera2 j10-cv-node
```

Use the `pytest` / `j10-cv-node` commands, not `python -m pytest` / `python -m cv_node`,
while your shell is inside this directory — same reason as `companion/mavlink_bridge`: see
[`../README.md`](../README.md) "Running it" and the `[tool.pytest.ini_options]` comment in
this package's `pyproject.toml`. `companion/cv_node/` (this package's project root) would
otherwise shadow the real, pip-installed `cv_node`.

To see it feeding the bridge for real: run `j10-cv-node` in one terminal and
`j10-mavlink-bridge` (from `companion/mavlink_bridge`) in another, pointed at the same
`shm_name` (the default, `j10_cv_cmd`, matches on both sides already) — no `cv_stub.py`
needed once this node exists; that stub was always meant to be replaced by this package,
not extended alongside it.

## Tuning

Every threshold lives in `CVNodeConfig` (`cv_node/config.py`), overridable via `J10_CV_*`
env vars — none of the defaults are calibrated against a real camera or a real indoor
space, they're reasonable starting points. In particular:

- **`blocked_density_threshold`** (default `0.12`) and **`clear_margin`** (default `0.02`)
  depend entirely on your lens, lighting, and what texture your test space actually has.
  Point the camera at a real "open" hallway and a real "blocked" obstacle, log the raw
  `ZoneDensities` (`log_level=DEBUG`), and set these from what you actually see — don't
  fly on the defaults.
- **`write_rate_hz`** (default `8.0`) is a target, not a measured number — profile Canny +
  blur at your actual `frame_width`/`frame_height` on the real Pi Zero 2W before trusting
  it. It's deliberately well under `mavlink_bridge`'s 20 Hz setpoint rate, mirroring the
  two-rate cascade already established for the PC-side architecture
  (`docs/ARCHITECTURE.md` §2): the slow layer decides direction, the fast layer (the
  bridge) guarantees the FC always has a fresh, bounded command regardless of how long a
  vision tick takes — including if this node stalls, in which case the bridge's own
  staleness failsafe takes over without this node having to do anything special.
- **Camera mount orientation** — `obstacle_avoidance.decide()` assumes the image's left
  column is the vehicle's left (forward-facing, unmirrored mount). A flipped or rotated
  mount silently inverts every turn decision; verify this on the bench (wave a hand on one
  side, confirm the logged `chosen_zone` and the sign of `yaw_rate` match) before flying.
