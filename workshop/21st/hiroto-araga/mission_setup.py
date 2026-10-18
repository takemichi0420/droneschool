from pymavlink import mavutil


def add_waypoint(mission, lat, lon, alt):
    # 新しいウェイポイントを追加
    new_waypoint = mavutil.mavlink.MAVLink_mission_item_int_message(
        master.target_system,            
        master.target_component,
        len(mission),
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0, 0, 0, 0, 0, 0,
        int(lat * 1e7),
        int(lon * 1e7),
        int(alt)
        )
    mission.append(new_waypoint)


def upload_mission(master, mission):
    # ミッションをアップロード
    master.mav.mission_clear_all_send(
        master.target_system, master.target_component)
    master.mav.mission_count_send(
        master.target_system, master.target_component, len(mission))
    for i, item in enumerate(mission):
        master.mav.send(item)

    # アップロードしたミッションを機体に設定
    master.mav.mission_set_current_send(0, master.target_component, 0)
    master.mav.mission_request_list_send(
        master.target_system, master.target_component)


def print_mission(mission):
    for element in mission:
        print(element)


if __name__ == '__main__':
    # 機体への接続
    master: mavutil.mavfile = mavutil.mavlink_connection(
        "tcp:127.0.0.1:5762", source_system=1, source_component=90)
    master.wait_heartbeat()

    # ミッション初期化
    master.mav.mission_clear_all_send(
        master.target_system, master.target_component
    )
    print("機体ミッション初期化完了")

    # ミッションリスト初期化
    downloaded_mission = []

    # ウェイポイントの追加
    add_waypoint(downloaded_mission, lat=35.8791732, lon=140.3357399, alt=5)
    add_waypoint(downloaded_mission, lat=35.8791732, lon=140.3357399, alt=5)
    add_waypoint(downloaded_mission, lat=35.8792134, lon=140.3356292, alt=5)
    add_waypoint(downloaded_mission, lat=35.8790966, lon=140.3355628, alt=5)
    add_waypoint(downloaded_mission, lat=35.8790612, lon=140.3356768, alt=5)
    print("ミッションへのウェイポイント追加完了")
    print_mission(downloaded_mission)

    # アップロード
    upload_mission(master, downloaded_mission)
    print("ミッションアップロード完了")
