from pymavlink import mavutil

device = "tcp:192.168.42.1:5762"

master = mavutil.mavlink_connection(device)
master.wait_heartbeat()

master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    10,  # 10 Hz
    1,   # 開始
)

while True:
    msg = master.recv_match(
        type=["GLOBAL_POSITION_INT", "SYS_STATUS", "ATTITUDE"], blocking=True
    )
    if msg is None:
        continue

    if msg.get_type() == "GLOBAL_POSITION_INT":
        print(f"Alt  : {msg.relative_alt / 1000:.1f} m")
        print(f"GPS  : ({msg.lat / 1e7:.6f}, {msg.lon / 1e7:.6f})")

    if msg.get_type() == "SYS_STATUS":
        print(f"Bat  : {msg.voltage_battery / 1000:.1f} V")
