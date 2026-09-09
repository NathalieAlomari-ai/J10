#!/usr/bin/env bash
#
# test1.sh — J10 PT1 bench-test setup helper.
#
# Pulls the latest main, (re)installs the three companion/ packages (j10_shm_protocol,
# mavlink_bridge, cv_node) in editable mode into one shared venv (companion/.venv), runs
# the test suite as a sanity check, and prints the exact commands to run the MAVLink
# bridge and CV node in two separate terminals.
#
# Usage:
#   ./test1.sh              full run: pull, install, test, print instructions
#   ./test1.sh --no-pull    skip the git pull (test whatever's currently checked out)
#   ./test1.sh --skip-tests skip the pytest sanity check
#
# Safety: this script only sets up software. It never arms the vehicle, changes flight
# mode, or touches the props. Read companion/README.md "Safety posture" and
# docs/ARCHITECTURE.md before connecting a battery — props OFF for this.

set -euo pipefail

# -- output helpers (plain text if not a tty, e.g. piped into a log file) -------------
if [[ -t 1 ]]; then
  BOLD=$'\033[1m'; RED=$'\033[31m'; GREEN=$'\033[32m'; YELLOW=$'\033[33m'; CYAN=$'\033[36m'; RESET=$'\033[0m'
else
  BOLD=''; RED=''; GREEN=''; YELLOW=''; CYAN=''; RESET=''
fi
info()   { printf '%s\n' "${CYAN}==>${RESET} $*"; }
ok()     { printf '%s\n' "${GREEN}OK${RESET}  $*"; }
warn()   { printf '%s\n' "${YELLOW}!!${RESET}  $*"; }
fail()   { printf '%s\n' "${RED}FAIL${RESET} $*" >&2; exit 1; }
header() { printf '\n%s\n' "${BOLD}== $* ==${RESET}"; }

usage() {
  cat <<'EOF'
Usage: ./test1.sh [--no-pull] [--skip-tests]

  --no-pull      skip the git pull, test whatever's currently checked out
  --skip-tests   skip the pytest sanity check after install
EOF
}

DO_PULL=1
DO_TESTS=1
for arg in "$@"; do
  case "$arg" in
    --no-pull) DO_PULL=0 ;;
    --skip-tests) DO_TESTS=0 ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; fail "unknown argument: $arg" ;;
  esac
done

# -- 0. repo root, sanity check we're actually in the J10 checkout --------------------
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_ROOT"
[[ -d companion/j10_shm_protocol && -d companion/mavlink_bridge && -d companion/cv_node ]] \
  || fail "this doesn't look like the J10 repo root — expected companion/{j10_shm_protocol,mavlink_bridge,cv_node} next to test1.sh"

header "J10 PT1 bench-test setup"
info "repo root: $REPO_ROOT"

# -- 1. pull latest main ---------------------------------------------------------------
if [[ "$DO_PULL" -eq 1 ]]; then
  header "1/4 -- pulling latest main"
  if ! git diff --quiet || ! git diff --cached --quiet; then
    fail "you have uncommitted changes -- commit, stash, or re-run with --no-pull. Refusing to touch your working tree."
  fi
  CURRENT_BRANCH="$(git rev-parse --abbrev-ref HEAD)"
  if [[ "$CURRENT_BRANCH" != "main" ]]; then
    info "currently on branch '$CURRENT_BRANCH', switching to main"
    git checkout main
  fi
  git pull --ff-only origin main \
    || fail "git pull --ff-only failed -- local main has diverged from origin/main. Resolve manually; this script won't force or rebase for you."
  ok "up to date: $(git rev-parse --short HEAD) $(git log -1 --format=%s)"
else
  warn "skipping git pull (--no-pull) -- testing whatever's currently checked out"
fi

# -- 2. venv ----------------------------------------------------------------------------
header "2/4 -- virtual environment"
VENV="$REPO_ROOT/companion/.venv"

IS_PI=0
if grep -qi "raspberry pi" /proc/device-tree/model 2>/dev/null; then
  IS_PI=1
  ok "Raspberry Pi detected"
else
  warn "not running on a Raspberry Pi (or couldn't tell) -- using a plain venv."
  warn "picamera2 specifically won't be installable off-Pi at all (expected -- see"
  warn "companion/cv_node/README.md). The 'opencv' camera backend still works fine"
  warn "for bench-testing cv_node's logic against a USB webcam on a dev machine."
fi

if [[ "$IS_PI" -eq 1 ]]; then
  if ! python3 -c "import cv2, numpy, picamera2" >/dev/null 2>&1; then
    warn "python3-opencv / python3-numpy / python3-picamera2 aren't all importable from"
    warn "the system Python. Needed (see companion/cv_node/README.md 'Installing on the Pi'):"
    echo "      sudo apt update && sudo apt install -y python3-opencv python3-picamera2 python3-numpy"
    if [[ -t 0 ]]; then
      read -r -p "    Install them now? [y/N] " REPLY
      if [[ "$REPLY" =~ ^[Yy]$ ]]; then
        sudo apt update && sudo apt install -y python3-opencv python3-picamera2 python3-numpy
        ok "apt packages installed"
      else
        warn "skipped -- cv_node's real (picamera2) camera backend won't work until these"
        warn "are installed. Continuing anyway; re-run this script after installing them."
      fi
    else
      warn "no terminal attached to prompt for sudo -- run that command yourself, then re-run this script."
    fi
  else
    ok "python3-opencv / numpy / picamera2 already importable from the system Python"
  fi
  VENV_FLAGS=(--system-site-packages)
else
  VENV_FLAGS=()
fi

if [[ ! -d "$VENV" ]]; then
  info "creating venv at companion/.venv"
  python3 -m venv "${VENV_FLAGS[@]}" "$VENV"
else
  ok "reusing existing venv at companion/.venv"
fi
# shellcheck disable=SC1091
source "$VENV/bin/activate"
ok "activated: $(python3 --version) at $(command -v python3)"

# -- 3. (re)install all three companion packages, in dependency order ------------------
header "3/4 -- installing companion packages (editable)"
pip install --quiet --upgrade pip

info "removing any previous editable installs for a clean reinstall"
pip uninstall -y j10-shm-protocol j10-mavlink-bridge j10-cv-node >/dev/null 2>&1 || true

info "1. j10_shm_protocol -- the shared-memory contract, everything else depends on it"
pip install -e "$REPO_ROOT/companion/j10_shm_protocol[dev]"

info "2. mavlink_bridge -- pymavlink/pyserial are lightweight, plain pip is fine everywhere"
pip install -e "$REPO_ROOT/companion[dev]"

info "3. cv_node"
if [[ "$IS_PI" -eq 1 ]]; then
  info "   on the Pi: --no-deps, relying on apt's opencv/numpy/picamera2 above"
  pip install -e "$REPO_ROOT/companion/cv_node" --no-deps
else
  pip install -e "$REPO_ROOT/companion/cv_node[dev]"
fi
ok "all three packages installed"

# -- 4. sanity check ---------------------------------------------------------------------
if [[ "$DO_TESTS" -eq 1 ]]; then
  header "4/4 -- running the test suite (no camera, no serial port, no hardware needed)"
  if pytest -v \
      "$REPO_ROOT/companion/j10_shm_protocol/tests" \
      "$REPO_ROOT/companion/tests" \
      "$REPO_ROOT/companion/cv_node/tests"; then
    ok "all tests passed"
  else
    warn "tests failed -- the install above may still be broken. Read the output before"
    warn "connecting a battery."
  fi
else
  warn "skipping test suite (--skip-tests)"
fi

# -- done: bench-test instructions -------------------------------------------------------
header "Ready -- bench test instructions"
cat <<EOF

${BOLD}Props OFF for this. Always.${RESET} See companion/README.md "Safety posture" and
docs/ARCHITECTURE.md (Phase 6/7) before connecting a battery.

Open ${BOLD}two${RESET} terminal windows on this Pi. In BOTH, first:

    cd $REPO_ROOT/companion
    source .venv/bin/activate

${BOLD}Terminal 1 -- the CV node${RESET} (Pi Camera Module 3):

    J10_CV_CAMERA_BACKEND=picamera2 j10-cv-node

${BOLD}Terminal 2 -- the MAVLink bridge${RESET} (Serial link to the CUAV V7 Nano):

    J10_BRIDGE_SERIAL_PORT=/dev/serial0 j10-mavlink-bridge

Watch Terminal 2's log for:
  "FC HEARTBEAT received: system=... component=..."   -> the serial link is good
  "failsafe cleared, resuming CV-commanded velocity"   -> the CV node's commands are
                                                           actually reaching the bridge

To deliberately test the failsafe (do this before anything flies): Ctrl-C Terminal 1
and confirm Terminal 2 logs "failsafe hover engaged: CV command stale (...)" within
about 250ms.

Full docs: companion/README.md and companion/cv_node/README.md.
EOF
