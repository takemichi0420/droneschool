from pymavlink import mavutil
import math
import time

# 機体への接続
master: mavutil.mavfile = mavutil.mavlink_connection(
    device="tcp:127.0.0.1:5762", source_system=1, source_component=90)
master.wait_heartbeat()


def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]


def send_attitude(roll, pitch, yaw, thrust=0.0):
    """姿勢(クオータニオン)で機体を制御するSET_ATTITUDE_TARGETを送信"""
    master.mav.set_attitude_target_send(
        0,      # ブートからの時間（今回は未使用）
        master.target_system, master.target_component,  # ターゲットシステム、コンポーネント
        # マスク: ロール/ピッチ/ヨーの各レートを無視し、姿勢(quaternion)で制御。
        # ※ArduCopterはレートを「全軸使う」か「全軸無視」のどちらかしか受け付けず、
        #   中途半端な指定(例:0b100)は無視されるため0b00000111にする。
        0b00000111,
        to_quaternion(roll, pitch, yaw),  # クオータニオン角度(ロール,ピッチ,ヨー角度)
        0, 0, 0,    # ロール,ピッチ,ヨーレート（マスクで無視）
        thrust)     # スラスト(GUID_OPTIONS既定では上昇率として解釈。0=高度維持)


# 姿勢制御は送り続ける必要があるため、一定時間ストリーム送信する。
# ① ロール20度・ピッチ-20度に傾ける（約3秒）
for _ in range(0, 60):
    send_attitude(20, -20, 0)
    time.sleep(0.05)  # 20Hzで送信

# ② 水平姿勢に戻す（約2秒）
for _ in range(0, 40):
    send_attitude(0, 0, 0)
    time.sleep(0.05)
