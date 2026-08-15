# LatencyReport — one pipeline stage's contribution to end-to-end latency.
#
# Every stage on the critical path emits one of these per item processed. The latency
# monitor aggregates them into p50/p95/p99 per stage. This is how the 300 ms budget gets
# proven rather than assumed.

std_msgs/Header header

# Emitting stage: "CAPTURE" | "TRANSPORT" | "DECODE" | "INFERENCE" | "CONTROL" | "SAFETY" | "BRIDGE"
string stage

# Capture time of the camera frame this work traces back to. The correlation key that
# lets the monitor stitch stages into one end-to-end measurement.
builtin_interfaces/Time source_stamp

# Wall-clock time spent inside this stage, milliseconds
float32 stage_latency_ms

# Elapsed time from source_stamp to this stage's completion, milliseconds.
# At the final stage this is the end-to-end number that must stay under budget.
float32 cumulative_latency_ms

# Monotonic per-stage sequence number, for detecting gaps the timing alone would hide
uint64 sequence
