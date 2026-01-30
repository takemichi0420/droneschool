from pymavlink import mavutil   
import time

master: mavutil.mavfile = mavutil.mavlink_connection(
    '127.0.0.1:14551', source_system=2, source_component=91)
master.wait_heartbeat()

print("Heartbeat from system (system %u component %u)" 
      % (master.target_system, master.target_component))

while True:
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
        0,
        0,
        0)
    time.sleep(1)
