# SafetyStatus — why the commanded velocity differs from the requested one.
#
# Published every safety filter cycle, not only on change, so a bag replay can always
# answer "what was the filter doing at time T".

# Escalation ladder
uint8 STATE_NOMINAL=0    # command passed through unmodified
uint8 STATE_LIMITED=1    # command clamped or shaped, still flying the intent
uint8 STATE_BRAKING=2    # commanding zero velocity, holding position
uint8 STATE_LANDING=3    # safety-initiated descent
uint8 STATE_ESTOP=4      # latched stop, requires explicit operator reset

std_msgs/Header header

# One of STATE_* above
uint8 state

# Names of every limit active this cycle, e.g. ["ALT_FLOOR", "GEOFENCE_X", "VLA_TIMEOUT"].
# Empty when state is STATE_NOMINAL.
string[] active_limits

# What the arbitration winner asked for, before filtering
geometry_msgs/Twist requested

# What was actually sent to the flight controller
geometry_msgs/Twist commanded

# False when the mission manager has not enabled autonomy, or a watchdog has revoked it
bool autonomy_enabled

# Which input won arbitration: "ESTOP" | "MANUAL" | "OVERRIDE" | "AUTONOMOUS"
string arbitration_source
