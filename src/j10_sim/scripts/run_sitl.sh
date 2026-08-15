#!/usr/bin/env bash
#
# Start ArduCopter SITL wired to the Gazebo iris model, with the J10 indoor parameter set.
#
# Requires an ArduPilot source tree (sim_vehicle.py lives there; it is not a ROS package).
#   git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
#   cd ardupilot && ./waf configure --board sitl && ./waf copter
#
# Gazebo must already be running the world, because ArduPilot connects to the
# ArduPilotPlugin's JSON backend rather than starting the simulator itself.

set -euo pipefail

ARDUPILOT_DIR="${ARDUPILOT_DIR:-${HOME}/ardupilot}"
PARAM_FILE="${PARAM_FILE:-}"
MAVROS_OUT="${MAVROS_OUT:-udp:127.0.0.1:14551}"
FRAME="${FRAME:-gazebo-iris}"
EXTRA_ARGS=()

usage() {
  cat <<'EOF'
Usage: run_sitl.sh [options] [-- extra sim_vehicle.py args]

Options:
  --ardupilot-dir DIR   ArduPilot source tree (default: $ARDUPILOT_DIR or ~/ardupilot)
  --param-file FILE     Parameter file to load (default: the installed sitl_indoor.parm)
  --mavros-out ENDPOINT MAVLink output for MAVROS (default: udp:127.0.0.1:14551)
  --frame FRAME         sim_vehicle.py frame (default: gazebo-iris)
  --console             Also open the MAVProxy console and map
  -h, --help            This message

Environment variables of the same name are honoured for each option.
EOF
}

CONSOLE=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --ardupilot-dir) ARDUPILOT_DIR="$2"; shift 2 ;;
    --param-file)    PARAM_FILE="$2"; shift 2 ;;
    --mavros-out)    MAVROS_OUT="$2"; shift 2 ;;
    --frame)         FRAME="$2"; shift 2 ;;
    --console)       CONSOLE=1; shift ;;
    -h|--help)       usage; exit 0 ;;
    --)              shift; EXTRA_ARGS=("$@"); break ;;
    *)               echo "run_sitl.sh: unknown option '$1'" >&2; usage >&2; exit 2 ;;
  esac
done

# Default the parameter file to the one installed alongside this package.
if [[ -z "${PARAM_FILE}" ]]; then
  # Ask the running ROS environment where j10_sim actually lives. This is robust to both
  # the default per-package install layout and `colcon build --merge-install`, where the
  # relative path from this script to share/ differs.
  if command -v ros2 >/dev/null 2>&1; then
    PKG_PREFIX="$(ros2 pkg prefix j10_sim 2>/dev/null || true)"
    if [[ -n "${PKG_PREFIX}" && -f "${PKG_PREFIX}/share/j10_sim/config/sitl_indoor.parm" ]]; then
      PARAM_FILE="${PKG_PREFIX}/share/j10_sim/config/sitl_indoor.parm"
    fi
  fi
fi

if [[ -z "${PARAM_FILE}" ]]; then
  SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  for candidate in \
    "${SCRIPT_DIR}/../../share/j10_sim/config/sitl_indoor.parm" \
    "${SCRIPT_DIR}/../config/sitl_indoor.parm"
  do
    if [[ -f "${candidate}" ]]; then
      PARAM_FILE="$(cd "$(dirname "${candidate}")" && pwd)/$(basename "${candidate}")"
      break
    fi
  done
fi

if [[ ! -d "${ARDUPILOT_DIR}" ]]; then
  echo "run_sitl.sh: ArduPilot tree not found at '${ARDUPILOT_DIR}'." >&2
  echo "             Clone it, or pass --ardupilot-dir / set ARDUPILOT_DIR." >&2
  exit 1
fi

SIM_VEHICLE="${ARDUPILOT_DIR}/Tools/autotest/sim_vehicle.py"
if [[ ! -x "${SIM_VEHICLE}" ]]; then
  echo "run_sitl.sh: ${SIM_VEHICLE} not found or not executable." >&2
  exit 1
fi

if [[ -z "${PARAM_FILE}" || ! -f "${PARAM_FILE}" ]]; then
  echo "run_sitl.sh: parameter file not found. Pass --param-file explicitly." >&2
  exit 1
fi

ARGS=(
  -v ArduCopter
  -f "${FRAME}"
  --model JSON
  --add-param-file="${PARAM_FILE}"
  --out="${MAVROS_OUT}"
)
if [[ "${CONSOLE}" -eq 1 ]]; then
  ARGS+=(--console --map)
fi
if [[ ${#EXTRA_ARGS[@]} -gt 0 ]]; then
  ARGS+=("${EXTRA_ARGS[@]}")
fi

echo "run_sitl.sh: ArduPilot   ${ARDUPILOT_DIR}"
echo "run_sitl.sh: parameters  ${PARAM_FILE}"
echo "run_sitl.sh: MAVROS out  ${MAVROS_OUT}"
echo

# sim_vehicle.py insists on being run from the ArduPilot tree for its build products.
cd "${ARDUPILOT_DIR}"
exec "${SIM_VEHICLE}" "${ARGS[@]}"
