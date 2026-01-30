from pymavlink import mavutil
import time

master: mavutil.mavfile = mavutil.mavlink_connection(
    device="127.0.0.1:14551", source_system=1, source_component=93
)
master.wait_heartbeat()

master.set_mode(4)
master.arducopter_arm()

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    10)

# master.set_mode(4)

# target_lon = 35.362938
# target_lat = 138.730456
# target_alt = 10

# master.mav.command_long_send(



# master.mav.set_position_target_global_int_send(
#     0,
#     master.target_system,
#     master.target_component,
#     mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
#     int(0b0000111111111000),
#     int(target_lat * 1e7),
#     int(target_lon * 1e7),
#     target_alt,
#     0,
#     0,
#     0,
#     0,
#     0,
#     0,
#     0,
#     0,
# )
