from pymavlink import mavutil
import  time

#Day2オンライン授業講師作成コード複写

#機体への接続
master: mavutil.mavfile = mavutil.mavlink_connection(
    "tcp:127.0.0.1:5762", source_system=1, source_component=90)

print("Waiting for heartbeat from system ... ")
master.wait_heartbeat() 

#Mission Planner Mavlink Inspectorでcomponent90の通信確認用(確認できず)
while True:
    print("Heartbeat received!")
    time.sleep(5)