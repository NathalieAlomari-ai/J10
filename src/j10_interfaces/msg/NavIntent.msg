# NavIntent — the VLA -> motion controller contract.
#
# A semantic navigation decision produced by the VLA model. Deliberately NOT a servo
# command: it carries a validity window so the consumer can expire it and decay to
# hover rather than repeating a stale command forever.

# Action taxonomy
uint8 ACTION_HOLD=0
uint8 ACTION_MOVE=1
uint8 ACTION_TURN=2
uint8 ACTION_EXPLORE=3
uint8 ACTION_LAND=4

std_msgs/Header header

# Natural-language instruction that produced this intent
string instruction

# One of ACTION_* above
uint8 action_type

# Desired velocity in the BODY frame (FLU: x forward, y left, z up).
# Consumers must treat this as a request, never as a command.
geometry_msgs/Twist velocity

# Seconds this intent remains valid from header.stamp. Once elapsed, the motion
# controller decays to hover. Must be > 0.
float32 duration

# Model confidence, 0.0-1.0. The safety filter may scale limits by this value.
float32 confidence

# Measured wall-clock inference time for this intent, milliseconds.
float32 inference_latency_ms

# Capture timestamp of the camera frame this decision was made from. Combined with
# header.stamp this yields true end-to-end perception latency.
builtin_interfaces/Time source_stamp

# Human-readable justification, for logs and post-flight review. Never parsed.
string rationale
