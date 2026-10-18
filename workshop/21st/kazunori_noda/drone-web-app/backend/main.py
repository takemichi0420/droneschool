"""
drone-web-app / backend / main.py
FastAPI + WebSocket + pymavlink  BlueOS Extension 対応版

  BlueOS 要件:
  - MAV_ENDPOINT 環境変数化（既定: udpout:host.docker.internal:14550）
  - 非ブロック接続（バックグラウンド daemon thread）
  - autopilot 特定 + mode_mapping_byname(hb.type) で Copter/Plane 誤判定を防止
  - 受信を target_system/target_component に限定（GCS HEARTBEAT 点滅防止）
  - ARM 判定を base_mode & MAV_MODE_FLAG_SAFETY_ARMED に統一
  - /register_service エンドポイント追加（avoid_iframes: True）
  - GET / が必ず 200 を返すことを保証
"""

import asyncio
import json
import logging
import os
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Optional

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pymavlink import mavutil

# ---------------------------------------------------------------------------
# ロガー設定
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("drone")

# ---------------------------------------------------------------------------
# パス解決
# ---------------------------------------------------------------------------
BASE_DIR     = os.path.dirname(os.path.abspath(__file__))
FRONTEND_DIR = os.path.join(BASE_DIR, "..", "frontend")

# ---------------------------------------------------------------------------
# MAVLink 定数
# ---------------------------------------------------------------------------
MAV_MODE_FLAG_SAFETY_ARMED        = 0x80
MAV_TYPE_GCS                      = 6
MAV_AUTOPILOT_INVALID             = 8
MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6
TYPE_MASK_POS_ONLY = 0b0000111111111000  # 位置のみ有効（速度・加速度・yaw・yaw_rate 無効）

GPS_FIX_LABELS = {
    0: "No GPS",  1: "No Fix",    2: "2D Fix",
    3: "3D Fix",  4: "DGPS",      5: "RTK Float",
    6: "RTK Fixed", 7: "Static",  8: "PPP",
}

# ---------------------------------------------------------------------------
# 環境変数（BlueOS では host.docker.internal 経由で Router に届く）
# ---------------------------------------------------------------------------
CONNECTION_STRING = os.environ.get(
    "MAV_ENDPOINT", "udpout:host.docker.internal:14550"
)

# ---------------------------------------------------------------------------
# 共有状態 + Lock
# ---------------------------------------------------------------------------
_state_lock = threading.Lock()

drone_state: dict = {
    "connected": False,
    "armed":     False,
    "mode":      "UNKNOWN",
    "latitude":  0.0,
    "longitude": 0.0,
    "altitude":  0.0,
    "heading":   0,
    "battery_voltage":   0.0,
    "battery_current":   0.0,
    "battery_remaining": -1,
    "gps_fix_type":    0,
    "gps_fix_label":   "No GPS",
    "gps_satellites":  0,
    "gps_hdop":        99.99,
    "gps_vdop":        99.99,
}

vehicle:          Optional[mavutil.mavfile] = None
autopilot_sysid:  Optional[int]            = None
autopilot_compid: Optional[int]            = None

# MODE_MAP: { "GUIDED": 4, "STABILIZE": 0, ... }  mode_mapping_byname で生成
MODE_MAP: dict[str, int] = {}

executor           = ThreadPoolExecutor(max_workers=4)
connected_clients: list[WebSocket] = []

# ---------------------------------------------------------------------------
# FastAPI アプリ
# ---------------------------------------------------------------------------
app = FastAPI(title="Drone Web App")
app.mount("/static", StaticFiles(directory=FRONTEND_DIR), name="static")


@app.get("/")
async def root():
    """GET / → 200 を保証（BlueOS Helper のヘルスチェック用）"""
    index = os.path.join(FRONTEND_DIR, "index.html")
    if os.path.exists(index):
        return FileResponse(index)
    # index.html が無い場合の最小フォールバック
    return JSONResponse({"status": "ok"})


@app.get("/register_service")
async def register_service():
    """
    BlueOS Helper が起動直後に呼ぶエンドポイント。
    avoid_iframes: True → WS 使用のため新ウィンドウで直接 http://<IP>:9999/ を開く。
    """
    return {
        "name":        "Drone Web App",
        "description": "pymavlink + FastAPI によるドローン Web コントロールパネル",
        "icon":        "mdi-drone",
        "company":     "kazunorinoda",
        "version":     "1.0.0",
        "webpage":     "",
        "api":         "/docs",
        "avoid_iframes": True,
    }


# ---------------------------------------------------------------------------
# WebSocket ブロードキャスト
# ---------------------------------------------------------------------------
async def broadcast_state():
    with _state_lock:
        snapshot = dict(drone_state)
    msg  = json.dumps({"type": "state", "state": snapshot})
    dead: list[WebSocket] = []
    for ws in connected_clients:
        try:
            await ws.send_text(msg)
        except Exception:
            dead.append(ws)
    for ws in dead:
        if ws in connected_clients:
            connected_clients.remove(ws)


async def broadcast_status(message: str):
    msg  = json.dumps({"type": "status", "message": message})
    dead: list[WebSocket] = []
    for ws in connected_clients:
        try:
            await ws.send_text(msg)
        except Exception:
            dead.append(ws)
    for ws in dead:
        if ws in connected_clients:
            connected_clients.remove(ws)


# ---------------------------------------------------------------------------
# MAVLink 受信ループ（connect スレッド内で継続実行）
# [BUG#3] set_mode_blocking との recv_match 競合を解消するため、
#          モード変更確認もここで完結させる。
# ---------------------------------------------------------------------------
def mavlink_recv_loop(loop: asyncio.AbstractEventLoop):
    global vehicle, drone_state

    logger.info("MAVLink 受信ループ開始")

    while True:
        if vehicle is None:
            break
        try:
            msg = vehicle.recv_match(blocking=True, timeout=0.1)
        except Exception as e:
            logger.warning(f"recv_match 例外: {e}")
            break

        if msg is None:
            continue

        # [BUG#3 / BlueOS要件③] 自機以外（GCS 等）のメッセージを無視
        if (msg.get_srcSystem()    != autopilot_sysid or
                msg.get_srcComponent() != autopilot_compid):
            continue

        msg_type = msg.get_type()
        changed  = False

        # ── GLOBAL_POSITION_INT ─────────────────────────────────────────
        if msg_type == "GLOBAL_POSITION_INT":
            new_lat = msg.lat / 1e7
            new_lon = msg.lon / 1e7
            new_alt = msg.relative_alt / 1000.0
            raw_hdg = msg.hdg
            with _state_lock:
                new_hdg = (raw_hdg // 100) if (raw_hdg != 65535) else drone_state["heading"]
                if (drone_state["latitude"]  != new_lat
                        or drone_state["longitude"] != new_lon
                        or drone_state["altitude"]  != new_alt
                        or drone_state["heading"]   != new_hdg):
                    drone_state["latitude"]  = new_lat
                    drone_state["longitude"] = new_lon
                    drone_state["altitude"]  = new_alt
                    drone_state["heading"]   = new_hdg
                    changed = True

        # ── HEARTBEAT ───────────────────────────────────────────────────
        elif msg_type == "HEARTBEAT":
            # [BlueOS要件③] ARM は base_mode & SAFETY_ARMED で判定
            new_armed = bool(msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
            rev_map   = {v: k for k, v in MODE_MAP.items()}
            new_mode  = rev_map.get(msg.custom_mode, f"MODE_{msg.custom_mode}")
            with _state_lock:
                if drone_state["armed"] != new_armed or drone_state["mode"] != new_mode:
                    drone_state["armed"] = new_armed
                    drone_state["mode"]  = new_mode
                    changed = True

        # ── SYS_STATUS ──────────────────────────────────────────────────
        elif msg_type == "SYS_STATUS":
            new_voltage   = msg.voltage_battery / 1000.0 if msg.voltage_battery != 65535 else 0.0
            new_current   = msg.current_battery / 100.0  if msg.current_battery  != -1    else -1.0
            new_remaining = int(msg.battery_remaining)
            with _state_lock:
                if (drone_state["battery_voltage"]   != new_voltage
                        or drone_state["battery_current"]   != new_current
                        or drone_state["battery_remaining"] != new_remaining):
                    drone_state["battery_voltage"]   = round(new_voltage,  2)
                    drone_state["battery_current"]   = round(new_current,  2)
                    drone_state["battery_remaining"] = new_remaining
                    changed = True

        # ── GPS_RAW_INT ─────────────────────────────────────────────────
        elif msg_type == "GPS_RAW_INT":
            fix_type   = int(msg.fix_type)
            fix_label  = GPS_FIX_LABELS.get(fix_type, f"FIX_{fix_type}")
            satellites = int(msg.satellites_visible) if msg.satellites_visible != 255 else 0
            hdop = msg.eph / 100.0 if msg.eph != 65535 else 99.99
            vdop = msg.epv / 100.0 if msg.epv != 65535 else 99.99
            with _state_lock:
                if (drone_state["gps_fix_type"]   != fix_type
                        or drone_state["gps_fix_label"]  != fix_label
                        or drone_state["gps_satellites"] != satellites
                        or drone_state["gps_hdop"]       != hdop
                        or drone_state["gps_vdop"]       != vdop):
                    drone_state["gps_fix_type"]   = fix_type
                    drone_state["gps_fix_label"]  = fix_label
                    drone_state["gps_satellites"] = satellites
                    drone_state["gps_hdop"]       = round(hdop, 2)
                    drone_state["gps_vdop"]       = round(vdop, 2)
                    changed = True

        if changed:
            asyncio.run_coroutine_threadsafe(broadcast_state(), loop)

    logger.info("MAVLink 受信ループ終了")


# ---------------------------------------------------------------------------
# MAVLink 接続（非ブロック・daemon thread）
# [BUG#5] executor を占有しないよう threading.Thread で独立起動
# [BlueOS要件②] autopilot 特定 + mode_mapping_byname で Copter/Plane 誤判定を防止
# ---------------------------------------------------------------------------
def _connect_loop(loop: asyncio.AbstractEventLoop):
    global vehicle, autopilot_sysid, autopilot_compid, MODE_MAP, drone_state

    while True:
        logger.info(f"MAVLink 接続試行: {CONNECTION_STRING}")
        asyncio.run_coroutine_threadsafe(
            broadcast_status(f"接続中: {CONNECTION_STRING}"), loop
        )

        try:
            m = mavutil.mavlink_connection(CONNECTION_STRING)

            # GCS として自分の HEARTBEAT を送信（Router が存在を認識するため）
            m.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0,
            )

            # autopilot（非GCS）の HEARTBEAT を最大30秒待つ
            hb      = None
            deadline = time.time() + 30
            while time.time() < deadline:
                raw = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
                if raw is None:
                    continue
                # GCS や Invalid autopilot は無視
                if raw.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                    continue
                if raw.type == MAV_TYPE_GCS:
                    continue
                hb = raw
                break

            if hb is None:
                logger.warning("Autopilot HEARTBEAT タイムアウト。再試行します")
                time.sleep(3)
                continue

            # autopilot を特定
            m.target_system    = hb.get_srcSystem()
            m.target_component = hb.get_srcComponent()

            # [BlueOS要件②] hb.type（MAV_TYPE）から Copter/Plane 対応の正しいモードマップを生成
            # mode_mapping() は直近 HB を見て誤判定することがあるため byname を使う
            MODE_MAP = mavutil.mode_mapping_byname(hb.type) or {}
            logger.info(
                f"Autopilot 確定: sysid={m.target_system} "
                f"compid={m.target_component} type={hb.type} "
                f"modes={list(MODE_MAP.keys())[:8]}..."
            )

            # データストリーム要求
            for stream_id, rate in [
                (mavutil.mavlink.MAV_DATA_STREAM_POSITION,        5),
                (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2),
                (mavutil.mavlink.MAV_DATA_STREAM_ALL,             4),
            ]:
                m.mav.request_data_stream_send(
                    m.target_system, m.target_component, stream_id, rate, 1
                )

            vehicle          = m
            autopilot_sysid  = m.target_system
            autopilot_compid = m.target_component

            with _state_lock:
                drone_state["connected"] = True

            asyncio.run_coroutine_threadsafe(broadcast_status("MAVLink 接続成功"), loop)
            asyncio.run_coroutine_threadsafe(broadcast_state(), loop)
            logger.info("MAVLink 接続成功 → 受信ループ開始")

            # 受信ループ（切断まで継続）
            mavlink_recv_loop(loop)

        except Exception as e:
            logger.error(f"接続例外: {e}")

        # 切断後のリセット
        vehicle          = None
        autopilot_sysid  = None
        autopilot_compid = None
        with _state_lock:
            drone_state["connected"] = False
            # 再接続時に古い値が残らないようリセット
            drone_state.update({
                "armed": False, "mode": "UNKNOWN",
                "battery_voltage": 0.0, "battery_current": 0.0, "battery_remaining": -1,
                "gps_fix_type": 0, "gps_fix_label": "No GPS",
                "gps_satellites": 0, "gps_hdop": 99.99, "gps_vdop": 99.99,
            })

        asyncio.run_coroutine_threadsafe(broadcast_status("MAVLink 切断。再接続します..."), loop)
        asyncio.run_coroutine_threadsafe(broadcast_state(), loop)
        time.sleep(3)


# ---------------------------------------------------------------------------
# コマンドハンドラー（executor スレッド内）
# [BUG#3] set_mode_blocking を廃止。モード変更コマンドのみ送信し確認は recv_loop に委譲。
# [BUG#1,6] goto の alt を float で渡す（1000倍しない）
# ---------------------------------------------------------------------------
def handle_command(data: dict, loop: asyncio.AbstractEventLoop):
    global vehicle

    cmd_type = data.get("type", "")

    def send_status(msg: str):
        asyncio.run_coroutine_threadsafe(broadcast_status(msg), loop)

    # ── connect（手動再接続。通常は起動時に自動接続済み）───────────────
    if cmd_type == "connect":
        if drone_state["connected"]:
            send_status("すでに接続済みです")
        else:
            send_status("接続はバックグラウンドで自動実行中です")
        return

    # 接続確認
    if not drone_state["connected"] or vehicle is None:
        send_status("エラー: 機体に未接続です")
        return

    # ── arm ─────────────────────────────────────────────────────────────
    if cmd_type == "arm":
        vehicle.mav.command_long_send(
            vehicle.target_system, vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0,
        )
        send_status("アームコマンド送信")

    # ── disarm ──────────────────────────────────────────────────────────
    elif cmd_type == "disarm":
        vehicle.mav.command_long_send(
            vehicle.target_system, vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0,
        )
        send_status("ディスアームコマンド送信")

    # ── takeoff ─────────────────────────────────────────────────────────
    elif cmd_type == "takeoff":
        altitude = float(data.get("altitude", 10.0))
        # GUIDED モードへ切替（コマンド送信のみ。確認は recv_loop の HEARTBEAT に委譲）
        if "GUIDED" in MODE_MAP:
            vehicle.set_mode(MODE_MAP["GUIDED"])
            send_status("GUIDED モード変更コマンド送信")
        else:
            send_status("警告: MODE_MAP に GUIDED が存在しません")
        vehicle.mav.command_long_send(
            vehicle.target_system, vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude,
        )
        send_status(f"離陸コマンド送信 (高度 {altitude}m)")

    # ── land ────────────────────────────────────────────────────────────
    elif cmd_type == "land":
        vehicle.mav.command_long_send(
            vehicle.target_system, vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0,
        )
        send_status("着陸コマンド送信")

    # ── goto ────────────────────────────────────────────────────────────
    elif cmd_type == "goto":
        lat = float(data.get("latitude",  0.0))
        lon = float(data.get("longitude", 0.0))
        alt = float(data.get("altitude",  10.0))  # [BUG#1,6] float のまま渡す
        if "GUIDED" in MODE_MAP:
            vehicle.set_mode(MODE_MAP["GUIDED"])
            send_status("GUIDED モード変更コマンド送信")
        else:
            send_status("警告: MODE_MAP に GUIDED が存在しません")
        vehicle.mav.set_position_target_global_int_send(
            0,
            vehicle.target_system, vehicle.target_component,
            MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            TYPE_MASK_POS_ONLY,
            int(lat * 1e7),
            int(lon * 1e7),
            alt,          # [BUG#1,6] メートル float（int(alt*1000) は誤り）
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0,
        )
        send_status(f"GoTo コマンド送信 ({lat:.6f}, {lon:.6f}, {alt}m)")

    # ── mode ────────────────────────────────────────────────────────────
    elif cmd_type == "mode":
        mode_name = data.get("mode", "")
        if not mode_name:
            send_status("エラー: モード名が指定されていません")
            return
        if mode_name not in MODE_MAP:
            send_status(f"エラー: 不明なモード '{mode_name}' / 利用可能: {list(MODE_MAP.keys())}")
            return
        vehicle.set_mode(MODE_MAP[mode_name])
        send_status(f"モード変更コマンド送信: {mode_name}")

    else:
        send_status(f"エラー: 不明なコマンド '{cmd_type}'")


# ---------------------------------------------------------------------------
# アプリ起動時に接続スレッドを自動開始
# ---------------------------------------------------------------------------
@app.on_event("startup")
async def startup_event():
    loop = asyncio.get_event_loop()
    t = threading.Thread(
        target=_connect_loop, args=(loop,), daemon=True, name="mavlink-connect"
    )
    t.start()
    logger.info(f"MAVLink 接続スレッド起動 → {CONNECTION_STRING}")


# ---------------------------------------------------------------------------
# WebSocket エンドポイント
# ---------------------------------------------------------------------------
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    connected_clients.append(websocket)
    logger.info(f"WebSocket 接続: {websocket.client}")

    # 接続直後に現在状態を即時送信
    with _state_lock:
        snapshot = dict(drone_state)
    await websocket.send_text(json.dumps({"type": "state", "state": snapshot}))

    loop = asyncio.get_event_loop()

    try:
        while True:
            raw = await websocket.receive_text()
            try:
                data = json.loads(raw)
            except json.JSONDecodeError:
                await websocket.send_text(
                    json.dumps({"type": "status", "message": "不正な JSON です"})
                )
                continue
            loop.run_in_executor(executor, handle_command, data, loop)

    except WebSocketDisconnect:
        logger.info(f"WebSocket 切断: {websocket.client}")
    except Exception as e:
        logger.warning(f"WebSocket エラー: {e}")
    finally:
        if websocket in connected_clients:
            connected_clients.remove(websocket)


# ---------------------------------------------------------------------------
# エントリーポイント
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=9999,
        reload=False,
        access_log=False,   # BlueOS Helper の継続ヘルスチェックでログ肥大を防ぐ
    )