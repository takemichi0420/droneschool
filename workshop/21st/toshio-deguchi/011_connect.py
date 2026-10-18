import time
from dronekit import connect

#デフォルト設定
# vehicle = connect('127.0.0.1:14551', wait_ready=True, timeout=60)
#養成塾指示設定（WindowsのIPv4アドレス）（失敗）
#vehicle = connect('tcp:192.168.11.2:5762', wait_ready=True, timeout=60)
#ChatGPT指示設定（wslのループバックアドレス）（成功）
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60)
#仮設の設定（powershellでipconfigコマンド実行で取得したWSLのイーサネットアダプタIPv4アドレス）(失敗)
#vehicle = connect('tcp:172.29.64.1:5762', wait_ready=True, timeout=60)
#仮設の設定（ubuntuで実行コマンドhostname -Iで取得したIPアドレス）（成功）
#vehicle = connect('tcp:172.29.66.78:5762', wait_ready=True, timeout=60)

while True:
    print ("====================================")
    print ("home_location: %s" % vehicle.home_location )
    print ("heading: %s" % vehicle.heading )
    print ("gimbal: %s" % vehicle.gimbal )
    print ("airspeed: %s" % vehicle.airspeed )
    print ("groundspeed: %s" % vehicle.groundspeed )
    print ("mode: %s" % vehicle.mode )
    print ("armed: %s" % vehicle.armed )
    time.sleep(1)
