from pymavlink import mavutil
import time
import sys

# 接続
master: mavutil.mavfile = mavutil.mavlink_connection(
    "tcp:127.0.0.1:5762", source_system=1, source_component=90)
master.wait_heartbeat()
print("接続完了")

# ★ 追加：RTL高度を5mに設定
rtl_alt_cm = 500  # 5m = 500cm
master.mav.param_set_send(
    master.target_system,
    master.target_component,
    b"RTL_ALT",
    float(rtl_alt_cm),
    mavutil.mavlink.MAV_PARAM_TYPE_REAL32
)
print("RTL高度を5mに設定しました")

# GUIDEDモードへ変更
mode_guided = "GUIDED"
master.set_mode_apm(master.mode_mapping()[mode_guided])

while True:
    if master.flightmode == mode_guided:
        break
    master.recv_msg()
print("GUIDEDモード変更完了")

# ARM
master.arducopter_arm()
master.motors_armed_wait()
print("アーム完了")

# 離陸
target_altitude = 5

master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, target_altitude)

# GLOBAL_POSITION_INT を 10Hz で受信
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0, 33, 100000, 0, 0, 0, 0, 0)

# 高度到達確認
while True:
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    current_altitude = msg.relative_alt / 1000.0
    print(f"高度: {current_altitude:.2f} m")

    if current_altitude >= target_altitude * 0.95:
        print("目標高度に到達")
        break

    time.sleep(0.1)


# AUTOモードへ変更（ミッション開始）
mode_auto = "AUTO"

if mode_auto not in master.mode_mapping():
    print("AUTOモードが見つかりません")
    exit(1)

mode_id = master.mode_mapping()[mode_auto]

master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id,
    0, 0, 0, 0, 0
)

# モード変更確認
while True:
    if master.flightmode == mode_auto:
        break
    master.recv_msg()

print("AUTOモードへ変更完了 → ミッション開始")

last_wp = 4   # ← あなたのミッションの最終WP番号に合わせて変更（例：5個なら4）

print(f"最終WP番号: {last_wp}")

while True:
    msg = master.recv_match(type=['MISSION_ITEM_REACHED', 'MISSION_CURRENT'], blocking=True)
    
    if msg.get_type() == 'MISSION_ITEM_REACHED':
        print(f"WP到達: {msg.seq}")
        if msg.seq == last_wp:
            print("すべてのWP通過 → RTLへ移行")
            break

    elif msg.get_type() == 'MISSION_CURRENT':
        print(f"現在のWP: {msg.seq}")
        if msg.seq == last_wp:
            print("最終WPに到達 → RTLへ移行")
            break

# 変更後のモード
mode = "RTL"

# モードが有効かをチェック
if mode not in master.mode_mapping():
    print('Unknown mode : {}'.format(mode))
    print('Try:', list(master.mode_mapping().keys()))
    sys.exit(1)

# モードIDを取得
mode_id = master.mode_mapping()[mode]

# モード変更リクエストを送信
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)

# モード変更の確認を行う
while True:
    if master.flightmode == mode:
        break
    master.recv_msg()

print("変更後モード:", master.flightmode)
