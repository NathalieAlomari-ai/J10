# StreamStatus — health of the incoming video link.
#
# The safety filter treats loss of video as loss of autonomy: no frames means the VLA is
# blind, so its intents must stop being trusted.

uint8 STATE_DISCONNECTED=0   # no frames ever received, or pipeline down
uint8 STATE_STREAMING=1      # frames arriving within the freshness window
uint8 STATE_DEGRADED=2       # frames arriving but late, dropping, or corrupt
uint8 STATE_LOST=3           # was streaming, now silent past the timeout

std_msgs/Header header

# One of STATE_* above
uint8 state

# Measured frame rate over the last second
float32 fps

# Seconds since the last successfully decoded frame
float32 time_since_last_frame

# Cumulative counters since node start
uint64 frames_received
uint64 frames_dropped        # dropped by the receiver because a newer frame was ready
uint64 decode_errors

# Negotiated stream geometry, 0 until the first frame decodes
uint32 width
uint32 height
