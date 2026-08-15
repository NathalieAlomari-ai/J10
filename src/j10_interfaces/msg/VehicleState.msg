# VehicleState — single coherent snapshot of the vehicle.
#
# Published by j10_mavlink, which is the ONLY package permitted to subscribe to /mavros/*.
# Every other node reads vehicle state from here. This is what makes a PX4 migration a
# one-package change instead of a rewrite.

std_msgs/Header header

# --- Link and arming ---
bool connected             # MAVLink heartbeat present
bool armed
bool guided                # FC is in a mode that accepts offboard setpoints
string mode                # raw FC mode string, e.g. "GUIDED", "LOITER", "LAND"

# --- Pose and motion (ENU local frame, EKF origin) ---
geometry_msgs/PoseStamped pose
geometry_msgs/TwistStamped velocity

# --- Indoor localization health ---
float32 rangefinder_range        # metres AGL from the MTF-01 LiDAR; NaN when invalid
bool rangefinder_valid
float32 flow_quality             # 0.0-1.0 optical flow confidence from the MTF-01
bool flow_valid
bool ekf_healthy                 # EKF3 reports usable position/velocity estimate

# --- Power ---
float32 battery_voltage
float32 battery_percentage       # 0.0-1.0, -1.0 when unknown

# Age of the newest underlying MAVROS message, seconds. Consumers use this rather than
# comparing clocks themselves.
float32 staleness_sec
