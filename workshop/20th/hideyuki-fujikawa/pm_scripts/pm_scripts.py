from pymavlink import mavutil
import time

master: mavutil.mavfile = mavutil.mavlink_connection(
    device="127.0.0.1:14551", source_system=1, source_component=93
)
master.wait_heartbeat()

master.set_mode_apm("GUIDED")
print("Current mode: {}".format(master.flightmode))

master.arducopter_arm()
master.motors_armed_wait()
print("ARMED")
time.sleep(5)

target_altitude = 10  # meters
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
    target_altitude)

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0, 33, 100000, 0, 0, 0, 0, 0)   

while True:
    recieved_msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    altitude = recieved_msg.relative_alt / 1000.0  # in meters
    print("Current Altitude: {:.2f} m".format(altitude))
    if altitude >= target_altitude * 0.95:
        print("Reached target altitude of {} meters".format(target_altitude))
        break

    time.sleep(0.1)


# mode = 'AUTO'

# if mode not in master.mode_mapping():
#     print('Unknown mode : {}'.format(mode))
#     print('Try:', list(master.mode_mapping().keys()))
#     exit(1) 

# mode_id = master.mode_mapping()[mode]

# master.mav.command_long_send(
#     master.target_system,
#     master.target_component,
#     mavutil.mavlink.MAV_CMD_DO_SET_MODE,
#     0,
#     1,
#     mode_id,
#     0,
#     0,
#     0,
#     0,
#     0,
# )

# while True:
#     if master.flightmode == mode:
#         print("Mode changed to {}".format(mode))
#         break
#     master.recv_match()

# print("Current mode: {}".format(master.flightmode))         


# # ARM
# master.arducopter_arm()
# master.motors_armed_wait()
# print("ARMED")
# time.sleep(5)   
# # DISARM
# master.arducopter_disarm()
# master.motors_disarmed_wait()
# print("DISARMED")





# master.mav.request_data_stream_send(
#     master.target_system, master.target_component,
#     0, 10, 1
# )

# master.mav.command_long_send(
#     master.target_system,
#     master.target_component,
#     mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
#     0, 33, 100000, 0, 0, 0, 0, 0)

# while True:
#     if (msg := master.recv_match()): print(msg.to_dict())
#     time.sleep(0.01)

# recieved_msg = master.recv_match(type='HEARTBEAT', blocking=True)
# print(recieved_msg)

# to_msg = master.mav.heartbeat_encode(
#     mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
#     mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
#     0, 0, 0)

# print(to_msg)
# master.mav.send(to_msg)

# master.mav.heartbeat_send(
#     mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
#     mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
#     0, 0, 0)

# print('target_system: {},  target_compornet: {}'.format(
#     master.target_system, master.target_component))

# while True:
#     time.sleep(1)

#     master.mav.heartbeat_send(
#         mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
#         mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
#         0, 0, 0)
