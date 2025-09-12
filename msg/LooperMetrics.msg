# neo_waypoint_follower/LooperMetrics.msg
std_msgs/Header header

# Progress
uint32 loop_idx                         # 0-based
uint32 wp_idx
string current_waypoint_name

# Distances (m)
float64 distance_from_start
float64 distance_from_last
float64 distance_remaining

# Timing (sec)
builtin_interfaces/Duration eta
builtin_interfaces/Duration navigation_time

# High-level looper state
uint8 looper_state                      # see enum below

# Enums
uint8 LOOPER_IDLE     = 0   # node up, not started (after boot or cancel)
uint8 LOOPER_RUNNING  = 1   # actively navigating a goal (or just sent)
uint8 LOOPER_WAITING  = 2   # between goals, in wait_at_waypoint delay
uint8 LOOPER_PAUSED   = 3   # user pressed Pause; goal canceled, indices kept
uint8 LOOPER_FINISHED = 4   # completed all loops/goals
uint8 LOOPER_ERROR    = 5   # unrecoverable (e.g., no YAML, server not up, abort with stop_on_failure)
