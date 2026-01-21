import math
import time
from pymavlink import mavutil

ALT = 10  # m
ALT_REACHED_HOLD_S = 5
ARRIVAL_RADIUS_M = 5.0
NAV_SEND_INTERVAL_S = 1.0
NAV_TIMEOUT_S = 300.0
MODE_TIMEOUT_S = 60.0
MODE_RETRIES = 3
ARM_TIMEOUT_S = 30.0
GLOBAL_POS_RETRY_S = 10.0
MODE_LOG_INTERVAL_S = 5.0
ACK_TIMEOUT_S = 3.0
GPS_FIX_TIMEOUT_S = 120.0
GPS_FIX_TYPE_MIN = 3
MODE_ID_OVERRIDES = {"GUIDED": 4}
READY_MIN_SYSIDS = 1

ORIGIN_LAT = 36.20561834925751
ORIGIN_LON = 136.38145829947558
HOME_LAT = 36.20647346306226
HOME_LON = 136.38307272897407

TARGET_LAT = 36.2058365
TARGET_LON = 136.3819063
SHAPE_RADIUS_M = 40.0
POLY_RADIUS_M = 60.0
CIRCLE_POINTS = 20
POLY_POINTS_PER_EDGE = 5
SHAPE_HOVER_S = 5.0

SYSIDS = list(range(1, 11))

def build_position_stage(sysids, positions, alt_m):
    if len(positions) != len(sysids):
        raise ValueError(f"positions={len(positions)} sysids={len(sysids)}")
    assignments = {}
    for sid, (lat, lon) in zip(sysids, positions):
        assignments[sid] = [(lat, lon, alt_m)]
    return assignments

def offset_latlon_m(lat, lon, north_m, east_m):
    dlat = north_m / 111111.0
    dlon = east_m / (111111.0 * math.cos(math.radians(lat)))
    return lat + dlat, lon + dlon

def build_circle_stage(sysids, center_lat, center_lon, alt_m, radius_m, points):
    assignments = {}
    n = len(sysids)
    for idx, sid in enumerate(sysids):
        phase = 2 * math.pi * idx / n
        waypoints = []
        for step in range(points + 1):
            theta = 2 * math.pi * step / points + phase
            north = radius_m * math.cos(theta)
            east = radius_m * math.sin(theta)
            lat, lon = offset_latlon_m(center_lat, center_lon, north, east)
            waypoints.append((lat, lon, alt_m))
        assignments[sid] = waypoints
    return assignments

def build_polygon_stage(sysids, center_lat, center_lon, alt_m, radius_m, sides, points_per_edge):
    assignments = {}
    n = len(sysids)
    vertices = []
    for i in range(sides):
        theta = 2 * math.pi * i / sides
        vertices.append((radius_m * math.cos(theta), radius_m * math.sin(theta)))
    base_points = []
    for i in range(sides):
        n1, e1 = vertices[i]
        n2, e2 = vertices[(i + 1) % sides]
        for step in range(points_per_edge):
            t = step / points_per_edge
            base_points.append((n1 + (n2 - n1) * t, e1 + (e2 - e1) * t))
    total_points = len(base_points)
    for idx, sid in enumerate(sysids):
        offset = int(total_points * idx / n)
        waypoints = []
        for step in range(total_points):
            north, east = base_points[(step + offset) % total_points]
            lat, lon = offset_latlon_m(center_lat, center_lon, north, east)
            waypoints.append((lat, lon, alt_m))
        north, east = base_points[offset]
        lat, lon = offset_latlon_m(center_lat, center_lon, north, east)
        waypoints.append((lat, lon, alt_m))
        assignments[sid] = waypoints
    return assignments

def build_stage_sequence(sysids):
    stages = []
    stages.append(
        (
            "MOVE",
            build_position_stage(sysids, [(TARGET_LAT, TARGET_LON)] * len(sysids), ALT),
        )
    )
    stages.append(
        (
            "CIRCLE",
            build_circle_stage(sysids, TARGET_LAT, TARGET_LON, ALT, SHAPE_RADIUS_M, CIRCLE_POINTS),
        )
    )
    stages.append(
        (
            "TRIANGLE",
            build_polygon_stage(sysids, TARGET_LAT, TARGET_LON, ALT, POLY_RADIUS_M, 3, POLY_POINTS_PER_EDGE),
        )
    )
    stages.append(
        (
            "SQUARE",
            build_polygon_stage(sysids, TARGET_LAT, TARGET_LON, ALT, POLY_RADIUS_M, 4, POLY_POINTS_PER_EDGE),
        )
    )
    return stages

def haversine_m(lat1, lon1, lat2, lon2):
    r = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return 2 * r * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def request_global_pos(sender, sysid, hz=5):
    interval_us = int(1_000_000 / hz)
    sender.mav.command_long_send(
        sysid, 1,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        interval_us,
        0, 0, 0, 0, 0
    )

def request_gps_raw(sender, sysid, hz=2):
    interval_us = int(1_000_000 / hz)
    sender.mav.command_long_send(
        sysid, 1,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
        interval_us,
        0, 0, 0, 0, 0
    )

def wait_heartbeats(receiver, sysids, timeout_s=10):
    deadline = time.time() + timeout_s
    remaining = set(sysids)
    while remaining and time.time() < deadline:
        msg = receiver.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if not msg:
            continue
        sid = msg.get_srcSystem()
        if sid in remaining:
            remaining.remove(sid)
    if remaining:
        raise TimeoutError(f"no heartbeat from sysids: {sorted(remaining)}")

def wait_gps_fix(receiver, sysids, timeout_s=GPS_FIX_TIMEOUT_S, fix_type_min=GPS_FIX_TYPE_MIN):
    deadline = time.time() + timeout_s
    remaining = set(sysids)
    last_log = time.time()
    while remaining and time.time() < deadline:
        msg = receiver.recv_match(type="GPS_RAW_INT", blocking=True, timeout=1)
        if not msg:
            if time.time() - last_log >= MODE_LOG_INTERVAL_S:
                print(f"waiting for GPS fix: remaining={len(remaining)}")
                last_log = time.time()
            continue
        sid = msg.get_srcSystem()
        if sid not in remaining:
            continue
        if msg.fix_type >= fix_type_min:
            remaining.remove(sid)
        if time.time() - last_log >= MODE_LOG_INTERVAL_S:
            print(f"waiting for GPS fix: remaining={len(remaining)}")
            last_log = time.time()
    if remaining:
        raise TimeoutError(f"GPS fix not reached for sysids: {sorted(remaining)}")

def get_mode_id(sender, mode_str):
    if mode_str in MODE_ID_OVERRIDES:
        return MODE_ID_OVERRIDES[mode_str]
    mode_map = sender.mode_mapping()
    if mode_str not in mode_map:
        raise RuntimeError(f"mode {mode_str} not in mapping")
    return mode_map[mode_str]

def set_mode(sender, sysid, mode_str):
    mode_id = get_mode_id(sender, mode_str)
    sender.mav.set_mode_send(
        sysid,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )

def set_mode_cmd(sender, sysid, mode_str):
    mode_id = get_mode_id(sender, mode_str)
    sender.mav.command_long_send(
        sysid, 1,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
        0, 0, 0, 0, 0
    )

def set_mode_with_ack(sender, receiver, sysid, mode_str, timeout_s=ACK_TIMEOUT_S):
    mode_id = get_mode_id(sender, mode_str)
    sender.mav.command_long_send(
        sysid, 1,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
        0, 0, 0, 0, 0
    )
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = receiver.recv_match(type="COMMAND_ACK", blocking=True, timeout=1)
        if not msg:
            continue
        if msg.get_srcSystem() != sysid:
            continue
        if msg.command != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            continue
        result = mavutil.mavlink.enums["MAV_RESULT"].get(msg.result, None)
        result_name = result.name if result else f"UNKNOWN({msg.result})"
        if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print(f"mode {mode_str} rejected by sysid {sysid}: {result_name}")
        return msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
    print(f"mode {mode_str} ack timeout from sysid {sysid}")
    return False

def collect_modes(receiver, sysids, mode_str, timeout_s=MODE_TIMEOUT_S):
    mode_id = get_mode_id(receiver, mode_str)
    deadline = time.time() + timeout_s
    remaining = set(sysids)
    reached = set()
    mode_counts = {}
    last_log = time.time()
    while remaining and time.time() < deadline:
        msg = receiver.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if not msg:
            if time.time() - last_log >= MODE_LOG_INTERVAL_S:
                print(f"waiting for {mode_str}: remaining={len(remaining)}")
                last_log = time.time()
            continue
        sid = msg.get_srcSystem()
        current_mode = mavutil.mode_string_v10(msg)
        mode_counts[current_mode] = mode_counts.get(current_mode, 0) + 1
        if sid not in remaining:
            continue
        if (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) and msg.custom_mode == mode_id:
            remaining.remove(sid)
            reached.add(sid)
        if time.time() - last_log >= MODE_LOG_INTERVAL_S:
            mode_summary = ", ".join(f"{k}={v}" for k, v in sorted(mode_counts.items()))
            print(f"waiting for {mode_str}: remaining={len(remaining)} modes={mode_summary}")
            last_log = time.time()
    return reached

def ensure_modes(sender, receiver, sysids, mode_str):
    remaining = set(sysids)
    reached = set()
    for _ in range(MODE_RETRIES):
        for sid in remaining:
            set_mode(sender, sid, mode_str)
            set_mode_cmd(sender, sid, mode_str)
            set_mode_with_ack(sender, receiver, sid, mode_str)
        newly = collect_modes(receiver, remaining, mode_str, timeout_s=MODE_TIMEOUT_S)
        reached |= newly
        remaining -= newly
        if not remaining:
            return reached
        print(f"retrying mode {mode_str} for {len(remaining)} vehicles")
    return reached

def arm(sender, sysid):
    sender.mav.command_long_send(
        sysid, 1,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )

def collect_armed(receiver, sysids, timeout_s=ARM_TIMEOUT_S):
    deadline = time.time() + timeout_s
    remaining = set(sysids)
    reached = set()
    while remaining and time.time() < deadline:
        msg = receiver.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if not msg:
            continue
        sid = msg.get_srcSystem()
        if sid not in remaining:
            continue
        if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            remaining.remove(sid)
            reached.add(sid)
    return reached

def ensure_armed(sender, receiver, sysids):
    remaining = set(sysids)
    reached = set()
    for _ in range(MODE_RETRIES):
        for sid in remaining:
            arm(sender, sid)
        newly = collect_armed(receiver, remaining, timeout_s=ARM_TIMEOUT_S)
        reached |= newly
        remaining -= newly
        if not remaining:
            return reached
        print(f"retrying arm for {len(remaining)} vehicles")
    return reached

def takeoff(sender, sysid, alt_m):
    sender.mav.command_long_send(
        sysid, 1,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0,
        alt_m
    )

def rtl(sender, sysid):
    sender.mav.command_long_send(
        sysid, 1,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0,
        0, 0, 0, 0,
        0, 0, 0
    )

def goto_latlon(sender, sysid, lat, lon, alt_m):
    sender.mav.set_position_target_global_int_send(
        0,
        sysid,
        1,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,
        int(lat * 1e7),
        int(lon * 1e7),
        alt_m,
        0, 0, 0,
        0, 0, 0,
        0, 0,
    )

def wait_altitudes(receiver, sender, sysids, target_alt_m, timeout_s=120):
    deadline = time.time() + timeout_s
    remaining = set(sysids)
    last_global_msg = time.time()
    while remaining and time.time() < deadline:
        msg = receiver.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
        if not msg:
            if time.time() - last_global_msg >= GLOBAL_POS_RETRY_S:
                for sid in sysids:
                    request_global_pos(sender, sid, hz=5)
                last_global_msg = time.time()
            continue
        last_global_msg = time.time()
        sid = msg.get_srcSystem()
        if sid not in remaining:
            continue
        rel_alt_m = msg.relative_alt / 1000.0
        if rel_alt_m >= target_alt_m:
            remaining.remove(sid)
    if remaining:
        raise TimeoutError(f"altitude not reached for sysids: {sorted(remaining)}")

def navigate_waypoints(receiver, sender, waypoints_by_sysid, arrival_radius_m, timeout_s):
    active = {
        sid: {"idx": 0, "last_send": 0.0}
        for sid, wps in waypoints_by_sysid.items()
        if wps
    }
    if not active:
        return
    start = time.time()
    while active:
        now = time.time()
        if timeout_s and now - start > timeout_s:
            raise TimeoutError(f"waypoint navigation timeout: {sorted(active.keys())}")
        for sid, state in list(active.items()):
            lat, lon, alt_m = waypoints_by_sysid[sid][state["idx"]]
            if now - state["last_send"] >= NAV_SEND_INTERVAL_S:
                goto_latlon(sender, sid, lat, lon, alt_m)
                state["last_send"] = now
        msg = receiver.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
        if not msg:
            continue
        sid = msg.get_srcSystem()
        state = active.get(sid)
        if not state:
            continue
        lat, lon, _alt_m = waypoints_by_sysid[sid][state["idx"]]
        clat = msg.lat / 1e7
        clon = msg.lon / 1e7
        dist = haversine_m(clat, clon, lat, lon)
        if dist <= arrival_radius_m:
            state["idx"] += 1
            if state["idx"] >= len(waypoints_by_sysid[sid]):
                del active[sid]
            else:
                state["last_send"] = 0.0

def main():
    # Mission Plannerと同じく 14550 を受信（sysid混在で入ってくる）
    receiver = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
    sender = mavutil.mavlink_connection("udpout:239.255.145.50:14550", source_system=255)
    receiver.wait_heartbeat()
    print("connected")

    # 最大10機（Mission Planner負荷を考慮）
    sysids = SYSIDS
    wait_heartbeats(receiver, sysids, timeout_s=10)
    for sid in sysids:
        request_global_pos(sender, sid, hz=5)
        request_gps_raw(sender, sid, hz=2)

    stages = build_stage_sequence(sysids)

    wait_gps_fix(receiver, sysids, timeout_s=GPS_FIX_TIMEOUT_S)

    # GUIDEDへ
    guided_sysids = ensure_modes(sender, receiver, sysids, "GUIDED")
    if len(guided_sysids) < READY_MIN_SYSIDS:
        raise TimeoutError("no vehicles entered GUIDED")
    if len(guided_sysids) != len(sysids):
        print(f"GUIDED ok={len(guided_sysids)} skipped={sorted(set(sysids)-set(guided_sysids))}")

    # ARM
    armed_sysids = ensure_armed(sender, receiver, guided_sysids)
    if len(armed_sysids) < READY_MIN_SYSIDS:
        raise TimeoutError("no vehicles armed")
    if len(armed_sysids) != len(guided_sysids):
        print(f"ARM ok={len(armed_sysids)} skipped={sorted(set(guided_sysids)-set(armed_sysids))}")

    # TAKEOFF（ほぼ同時）
    t0 = time.time()
    for sid in armed_sysids:
        takeoff(sender, sid, ALT)

    print(f"sent takeoff to {sorted(armed_sysids)} at +{time.time()-t0:.3f}s")
    wait_altitudes(receiver, sender, armed_sysids, ALT, timeout_s=120)
    time.sleep(ALT_REACHED_HOLD_S)
    for idx, (stage_name, stage_waypoints) in enumerate(stages):
        active_waypoints = {sid: wps for sid, wps in stage_waypoints.items() if sid in armed_sysids}
        print(f"stage {stage_name}: moving {len(active_waypoints)} vehicles")
        navigate_waypoints(receiver, sender, active_waypoints, ARRIVAL_RADIUS_M, NAV_TIMEOUT_S)
        if stage_name in {"CIRCLE", "TRIANGLE", "SQUARE"}:
            print(f"stage {stage_name}: hover {SHAPE_HOVER_S:.0f}s")
            time.sleep(SHAPE_HOVER_S)
    for sid in armed_sysids:
        rtl(sender, sid)
    print(f"sent RTL to {sorted(armed_sysids)}")

if __name__ == "__main__":
    main()
