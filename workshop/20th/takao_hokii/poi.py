from pymavlink import mavutil
from dataclasses import dataclass
import math
from typing import List, Tuple
import time


# ==============================
# 1. 構造物スペック
# ==============================
@dataclass
class StructureSpec:
    center_lat: float      # 構造物中心（緯度）
    center_lon: float      # 構造物中心（経度）
    width_m: float         # 構造物幅（東西方向）
    depth_m: float         # 構造物奥行き（南北方向）
    base_alt_m: float      # 基準高度
    height_m: float        # 構造物高さ

# WGS84 地球半径
R_EARTH = 6378137.0  # [m]

@dataclass
class GazeboOffset:
    x_m: float  # Gazebo座標系 X [m] （東が＋）
    y_m: float  # Gazebo座標系 Y [m] （北が＋）


def gazebo_xy_to_latlon(home_lat, home_lon, offset: GazeboOffset):
    """
    Gazeboの相対座標 (x: East, y: North) から
    構造物の緯度経度を近似計算する。
    """
    lat0 = home_lat
    lon0 = home_lon
    x = offset.x_m
    y = offset.y_m

    # 緯度方向の変位 [rad]
    dlat_rad = y / R_EARTH
    # 経度方向の変位 [rad]
    lat0_rad = math.radians(lat0)
    dlon_rad = x / (R_EARTH * math.cos(lat0_rad))

    # 変位をdegにして加算
    lat = lat0 + math.degrees(dlat_rad)
    lon = lon0 + math.degrees(dlon_rad)

    return lat, lon


# ==============================
# 2. ウェイポイント生成
# ==============================
def estimate_orbit_radius(spec: StructureSpec,
                          safety_margin_m: float = 1.0,
                          min_radius_m: float = 1.0) -> float:
    """
    構造物を完全に包み込む最小の円を作り、そこに安全マージンを足した半径を計算する。
    """
    half_diag = math.sqrt((spec.width_m / 2)**2 + (spec.depth_m / 2)**2)
    r = half_diag + safety_margin_m
    return max(r, min_radius_m)


def plan_vertical_levels(spec: StructureSpec,
                         start_alt_m: float = 1.0,   
                         end_alt_margin_m: float = 0.0, 
                         alt_step_m: float = 1.0,      
                         max_rings: int = 30) -> List[float]:
    """
    start_alt_m から 構造物高さまで alt_step_m 刻みで高度リングを作る。
    """
    bottom_alt = spec.base_alt_m + start_alt_m
    top_alt = spec.base_alt_m + spec.height_m + end_alt_margin_m

    if top_alt <= bottom_alt:
        return [bottom_alt]

    n_rings = int(math.floor((top_alt - bottom_alt) / alt_step_m)) + 1
    n_rings = max(1, min(n_rings, max_rings))  

    if n_rings == 1:
        return [bottom_alt]

    step = (top_alt - bottom_alt) / (n_rings - 1)

    return [bottom_alt + i * step for i in range(n_rings)]


def make_circle_points(center_lat: float,
                       center_lon: float,
                       radius_m: float,
                       n_points: int) -> List[Tuple[float, float]]:
    """
    指定した中心点（緯度・経度）のまわりに半径ｒの円を描き、
    その円周上の点を緯度経度で並べて返す。
    """
    lat0_rad = math.radians(center_lat)

    pts = []
    for i in range(n_points):
        theta = 2 * math.pi * i / n_points
        dx = radius_m * math.cos(theta)  # 東
        dy = radius_m * math.sin(theta)  # 北

        dlat = dy / R_EARTH
        dlon = dx / (R_EARTH * math.cos(lat0_rad))

        lat = center_lat + math.degrees(dlat)
        lon = center_lon + math.degrees(dlon)
        pts.append((lat, lon))

    return pts


def get_home_position(master, timeout=10.0):
    """
    HOME_POSITION メッセージからホームの緯度経度を取得する。
    """
    t0 = time.time()

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
        0, 0, 0, 0, 0, 0, 0, 0
    )

    while time.time() - t0 < timeout:
        msg = master.recv_match(type='HOME_POSITION', blocking=True, timeout=1)
        if msg:
            lat = msg.latitude / 1e7
            lon = msg.longitude / 1e7
            alt = msg.altitude / 1000.0  # mm → m

            return lat, lon, alt

    raise RuntimeError("Timeout waiting for HOME_POSITION")


def get_current_latlon(master, timeout=10.0):
    """
    現在地(lat, lon)を GLOBAL_POSITION_INT から取得する。
    """
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7

            return lat, lon
        
    raise RuntimeError("Timeout: cannot get GLOBAL_POSITION_INT for current lat/lon")


def approx_dist_m(lat1, lon1, lat2, lon2):
    """
    2点間の緯度経度から平面近似で距離[m]を計算する。
    """
    lat1r = math.radians(lat1)
    lat2r = math.radians(lat2)
    dlat = lat2r - lat1r
    dlon = math.radians(lon2 - lon1)
    x = dlon * math.cos((lat1r + lat2r) / 2.0) * R_EARTH
    y = dlat * R_EARTH

    return math.hypot(x, y)


def rotate_points_to_closest(circle_pts, cur_lat, cur_lon):
    """
   構造物との衝突回避のため、現在地に最も近いポイントから始まるように設定する。
    """
    if not circle_pts:
        return circle_pts

    best_i = 0
    best_d = float("inf")
    for i, (lat, lon) in enumerate(circle_pts):
        d = approx_dist_m(cur_lat, cur_lon, lat, lon)
        if d < best_d:
            best_d = d
            best_i = i

    rotated = circle_pts[best_i:] + circle_pts[:best_i]

    return rotated


def plan_structure_orbits(spec: StructureSpec,
                          cur_lat: float,
                          cur_lon: float,
                          n_points_per_ring: int = 36,
                          safety_margin_m: float = 10.0,
                          min_radius_m: float = 20.0):
    """
    指定した構造物スペックに基づき、複数高度の円周リングを計画する。
    """
    radius_m = estimate_orbit_radius(spec, safety_margin_m, min_radius_m)

    alt_list = plan_vertical_levels(spec, radius_m)

    base_circle = make_circle_points(spec.center_lat, spec.center_lon,
                                     radius_m, n_points_per_ring)
    base_circle = rotate_points_to_closest(base_circle, cur_lat, cur_lon)

    rings = []
    for alt in alt_list:
        ring = [(lat, lon, alt) for (lat, lon) in base_circle]  
        rings.append(ring)

    return radius_m, rings


# ==============================
# 3. ミッション生成
# ==============================
def make_mission_item_int(master, seq,
                          frame, command, current, autocontinue,
                          p1, p2, p3, p4, x, y, z):
    """
    MAVLink_mission_item_int_message を作るヘルパー関数
    """
    return mavutil.mavlink.MAVLink_mission_item_int_message(
        target_system=master.target_system,
        target_component=master.target_component,
        seq=seq,
        frame=frame,
        command=command,
        current=current,
        autocontinue=autocontinue,
        param1=p1,
        param2=p2,
        param3=p3,
        param4=p4,
        x=x,
        y=y,
        z=z
    )


def build_orbit_mission(master, spec: StructureSpec,
                        n_points_per_ring=36,
                        safety_margin_m=10.0,
                        min_radius_m=5.0):

    """
    指定した構造物スペックに基づき、ミッションアイテムのリストを生成する。
    """
    cur_lat, cur_lon = get_current_latlon(master, timeout=10.0)
    print(f"[POS] current lat/lon = {cur_lat:.7f}, {cur_lon:.7f}")

    radius_m, rings = plan_structure_orbits(
        spec,
        cur_lat=cur_lat,
        cur_lon=cur_lon,
        n_points_per_ring=n_points_per_ring,
        safety_margin_m=safety_margin_m,
        min_radius_m=min_radius_m
    )

    mission = []
    seq = 0

    # --- TAKEOFF --
    takeoff_alt = rings[0][0][2]  # 最下段の高度まで上げる
    mission.append(
        make_mission_item_int(
            master, seq,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            current=1,
            autocontinue=1,
            p1=0, p2=0, p3=0, p4=0,
            x=int(spec.center_lat * 1e7),
            y=int(spec.center_lon * 1e7),
            z=takeoff_alt
        )
    )
    seq += 1

    # --- DO_SET_ROI ---
    roi_alt = spec.base_alt_m + spec.height_m / 2.0
    mission.append(
        make_mission_item_int(
            master, seq,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            command=mavutil.mavlink.MAV_CMD_DO_SET_ROI,
            current=0,
            autocontinue=1,
            p1=3,      # MAV_ROI_LOCATION
            p2=0, p3=0, p4=0,
            x=int(spec.center_lat * 1e7),
            y=int(spec.center_lon * 1e7),
            z=roi_alt
        )
    )
    seq += 1

    # --- 各リングの円周ウェイポイント---
    for ring_idx, ring in enumerate(rings):
        for (lat, lon, alt) in ring:
            mission.append(
                make_mission_item_int(
                    master, seq,
                    frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    current=0,
                    autocontinue=1,
                    p1=0,
                    p2=2.0,  
                    p3=0,
                    p4=float("nan"),  
                    x=int(lat * 1e7),
                    y=int(lon * 1e7),
                    z=alt
                )
            )
            seq += 1

    # --- RTL ---
    mission.append(
        make_mission_item_int(
            master, seq,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            command=mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            current=0,
            autocontinue=1,
            p1=0, p2=0, p3=0, p4=0,
            x=0, y=0, z=0   
        )
    )

    print(f"[PLAN] radius={radius_m:.1f} m, rings={len(rings)}, "
          f"total_items={len(mission)}")

    return mission


# ==============================
# 4. ミッションアップロード 
# ==============================
def upload_mission(master, mission_items):
    """
    ミッションアイテムのリストをアップロードする。
    """

    n = len(mission_items)
    if n == 0:
        print("[MISSION] empty mission, skip")
        return

    target_sys = master.target_system or 1
    target_comp = master.target_component

    while True:
        msg = master.recv_match(
            type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'],
            blocking=False
        )
        if not msg:
            break

    # 既存ミッション削除
    master.mav.mission_clear_all_send(target_sys, target_comp)
    time.sleep(0.2)

    # カウント送信
    master.mav.mission_count_send(target_sys, target_comp, n)
    print(f"[MISSION] count={n} sent, waiting requests...")

    sent = 0
    while True:
        msg = master.recv_match(
            type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'],
            blocking=True,
            timeout=30
        )
        if msg is None:
            raise RuntimeError("Timeout waiting MISSION_REQUEST / MISSION_ACK")

        mtype = msg.get_type()

        if mtype in ('MISSION_REQUEST', 'MISSION_REQUEST_INT'):
            seq = msg.seq
            print(f"[MISSION] request seq={seq}")
            if not (0 <= seq < n):
                raise RuntimeError(f"Invalid mission request seq={seq}")
            master.mav.send(mission_items[seq])
            sent += 1

        elif mtype == 'MISSION_ACK':
            print(f"[MISSION] ACK received: type={msg.type}, sent={sent}/{n}")
            if sent < n:
                print("[MISSION] premature ACK (probably from clear_all), ignore and continue")
                continue
            else:
                break

    print(f"[MISSION] uploaded {sent}/{n} items")

    master.mav.mission_set_current_send(target_sys, target_comp, 0)
    print("[MISSION] set current seq=0")


# ==============================
# 5. AUTO開始
# ==============================
def set_rtl_params(master,
                   rtl_alt_m=10.0,
                   land_speed_cms=50):
    """
    RTL と着陸関連パラメータを設定する
    """
    print("[PARAM] setting RTL / LAND params")

    # RTL_ALT は cm 単位
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        b"RTL_ALT",
        rtl_alt_m * 100,  # m → cm
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )

    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        b"RTL_ALT_FINAL",
        0,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )

    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        b"RTL_AUTOLAND",
        1,
        mavutil.mavlink.MAV_PARAM_TYPE_INT8
    )

    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        b"LAND_ENABLE",
        1,
        mavutil.mavlink.MAV_PARAM_TYPE_INT8
    )

    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        b"LAND_SPEED",
        land_speed_cms,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )

    time.sleep(1.0)


def set_mode_blocking(master, mode_name: str):
    """
    モード変更をリクエストし、変更完了まで待つ。
    """
    modes = master.mode_mapping()
    if mode_name not in modes:
        raise RuntimeError(f"Mode {mode_name} not in mode_mapping(): {modes}")
    mode_id = modes[mode_name]

    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

    while True:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        if not msg:
            print("[MODE] waiting HEARTBEAT...")
            continue
        current_mode = mavutil.mode_string_v10(msg)
        # print("[MODE] now:", current_mode)
        if current_mode == mode_name:
            print("[MODE] ->", current_mode)
            break


def disable_arming_check(master):
    """
    ARMING_CHECK をオフにする。
    """
    print("[PARAM] disable ARMING_CHECK")
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        b"ARMING_CHECK",
        0,
        mavutil.mavlink.MAV_PARAM_TYPE_INT32
    )
    time.sleep(1.0)


def print_statustext(master, timeout=3.0):
    """
    STATUSTEXT メッセージを timeout 秒間受信して表示する。
    """
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = master.recv_match(type='STATUSTEXT', blocking=False)
        if msg:
            print("[STATUSTEXT]", msg.severity, msg.text)


def wait_for_position(master, timeout=30.0):
    """
    'Need Position Estimate' を避けるため、GLOBAL_POSITION_INT が来るまで待つ。
    """
    print("[POS] waiting for GLOBAL_POSITION_INT ...")
    t0 = time.time()
    got_pos = False
    while time.time() - t0 < timeout:
        msg = master.recv_match(
            type=['GLOBAL_POSITION_INT', 'LOCAL_POSITION_NED', 'STATUSTEXT'],
            blocking=False
        )
        if not msg:
            time.sleep(0.2)
            continue
        if msg.get_type() == 'GLOBAL_POSITION_INT':
            alt = msg.alt / 1000.0
            print(f"[POS] got GLOBAL_POSITION_INT alt={alt:.1f} m")
            got_pos = True
            break
        elif msg.get_type() == 'STATUSTEXT':
            print("[STATUSTEXT]", msg.severity, msg.text)

    if not got_pos:
        print("[POS] WARNING: no GLOBAL_POSITION_INT within timeout")


def guided_takeoff(master, alt=1.0):
    """
    GUIDED モードで離陸する。
    """
    set_mode_blocking(master, "GUIDED")

    print("[ARM] arm request...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print("[ARM] armed OK")

    print(f"[TAKEOFF] to {alt} m (GUIDED)")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,  
        0, 0,       
        alt
    )

    t0 = time.time()
    while time.time() - t0 < 20:
        msg = master.recv_match(
            type=['GLOBAL_POSITION_INT', 'STATUSTEXT'],
            blocking=False
        )
        if msg and msg.get_type() == 'GLOBAL_POSITION_INT':
            rel_alt = msg.relative_alt / 1000.0
            print(f"[POS] rel_alt={rel_alt:.1f} m")
        time.sleep(0.5)


def arm_and_start_auto(master, takeoff_alt=1.0):
    """
    ミッションを AUTO モードで開始する。
    """
    set_rtl_params(
        master,
        rtl_alt_m=5.0,    
        land_speed_cms=50
    )

    # disable_arming_check(master)
    print_statustext(master, 2.0)

    # 位置推定が安定するまで待つ
    wait_for_position(master, timeout=30.0)

    # GUIDED で離陸
    guided_takeoff(master, alt=takeoff_alt)

    # AUTO に切り替えてミッション開始
    set_mode_blocking(master, "AUTO")
    print_statustext(master, 3.0)

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    print("[AUTO] mission started (GUIDED→AUTO)")


# ==============================
# 6. メイン処理
# ==============================
def main():
    # --- 機体接続 ---
    master = mavutil.mavlink_connection("udp:127.0.0.1:14550")
    master.wait_heartbeat()
    print("[SYS] heartbeat from system",
          master.target_system, master.target_component)

    # --- Gazebo上の物体位置設定 ---
    offset = GazeboOffset(
        x_m=1.0,   
        y_m=1.0,  
    )

    # --- 構造物の設定---
    home_lat, home_lon, _ = get_home_position(master)
    obj_lat, obj_lon = gazebo_xy_to_latlon(home_lat, home_lon, offset)

    spec = StructureSpec(
        center_lat=obj_lat,
        center_lon=obj_lon,
        width_m=1.0,
        depth_m=1.0,
        base_alt_m=0.0,
        height_m=4.0
    )

    # --- ミッション構築 ---
    mission = build_orbit_mission(
        master,
        spec,
        n_points_per_ring=10,
        safety_margin_m=1.0,
        min_radius_m=1.0
    )

    # --- ミッションアップロード ---
    upload_mission(master, mission)

    # --- AUTO で開始 ---
    arm_and_start_auto(master)


if __name__ == "__main__":
    main()