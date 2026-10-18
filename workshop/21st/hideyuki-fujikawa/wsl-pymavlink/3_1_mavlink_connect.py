from pymavlink import mavutil

# SITL に UDP で接続
master = mavutil.mavlink_connection("tcp:127.0.0.1:5762")

# 最初の HEARTBEAT を待つ（接続確認）
master.wait_heartbeat()
print("接続成功")
print(f"  System ID  : {master.target_system}")
print(f"  Component  : {master.target_component}")