"""Drone Web App バックエンド。

FastAPI + WebSocket + pymavlink による最小構成のドローンWeb制御サーバー。
起動例: uvicorn main:app --port 9999 --reload
"""

import asyncio
import json
import logging
import time
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pymavlink import mavutil

CONNECTION_STRING = "tcp:127.0.0.1:5762"
HEARTBEAT_TIMEOUT = 30.0  # 接続時に HEARTBEAT を待つ最大秒数
GUIDED_WAIT_SEC = 5.0     # takeoff/goto 前の GUIDED 切替を待つ最大秒数
STREAM_RATE_HZ = 4
ALLOWED_MODES = ("GUIDED", "AUTO", "RTL", "LOITER", "STABILIZE")

FRONTEND_DIR = Path(__file__).resolve().parent.parent / "frontend"

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
logger = logging.getLogger("drone-webapp")

app = FastAPI(title="Drone Web App")
app.mount("/static", StaticFiles(directory=str(FRONTEND_DIR)), name="static")

# 機体状態。テレメトリー受信で更新し、更新のたびに全クライアントへ配信する。
state = {
    "connected": False,
    "armed": False,
    "mode": "UNKNOWN",
    "latitude": 0.0,
    "longitude": 0.0,
    "altitude": 0.0,
    "heading": 0,
}

vehicle = None        # mavutil の接続オブジェクト
receive_task = None   # MAVLink 受信タスク
clients = set()       # 接続中の WebSocket クライアント
connect_lock = asyncio.Lock()


@app.get("/")
async def index():
    return FileResponse(FRONTEND_DIR / "index.html")


# ---------------------------------------------------------------- MAVLink

def is_autopilot_heartbeat(msg):
    """GCS など機体以外の HEARTBEAT を除外する（チラつき防止）。"""
    return (
        msg.type != mavutil.mavlink.MAV_TYPE_GCS
        and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID
    )


def _connect_blocking():
    """機体へ接続し、本物のオートパイロットの HEARTBEAT を待つ（別スレッドで実行）。"""
    v = mavutil.mavlink_connection(CONNECTION_STRING)
    deadline = time.time() + HEARTBEAT_TIMEOUT
    while time.time() < deadline:
        hb = v.wait_heartbeat(timeout=max(0.1, deadline - time.time()))
        if hb is None:
            break
        if is_autopilot_heartbeat(hb):
            # 採用した発生源をコマンド送信先にする
            v.target_system = hb.get_srcSystem()
            v.target_component = hb.get_srcComponent()
            return v
    v.close()
    raise TimeoutError("オートパイロットの HEARTBEAT を受信できませんでした")


async def connect_vehicle():
    global vehicle, receive_task
    loop = asyncio.get_running_loop()
    vehicle = await loop.run_in_executor(None, _connect_blocking)

    # 位置系データストリームを要求
    vehicle.mav.request_data_stream_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,
        STREAM_RATE_HZ,
        1,
    )

    state["connected"] = True
    if receive_task is None or receive_task.done():
        receive_task = asyncio.create_task(receive_loop())
    logger.info(
        "vehicle connected: %s (sys=%d comp=%d)",
        CONNECTION_STRING, vehicle.target_system, vehicle.target_component,
    )


def mode_name_from_custom_mode(custom_mode):
    """mode_mapping()（モード名 -> ID）を逆引きしてモード名を得る。"""
    mapping = vehicle.mode_mapping() if vehicle is not None else None
    if not mapping:
        return "UNKNOWN"
    for name, mode_id in mapping.items():
        if mode_id == custom_mode:
            return name
    return "UNKNOWN"


def handle_mavlink_message(msg):
    """受信メッセージで状態を更新する。状態が変わったら True を返す。"""
    msg_type = msg.get_type()

    if msg_type == "HEARTBEAT":
        if not is_autopilot_heartbeat(msg):
            return False
        vehicle.target_system = msg.get_srcSystem()
        vehicle.target_component = msg.get_srcComponent()
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        mode = mode_name_from_custom_mode(msg.custom_mode)
        changed = armed != state["armed"] or mode != state["mode"]
        state["armed"] = armed
        state["mode"] = mode
        return changed

    if msg_type == "GLOBAL_POSITION_INT":
        state["latitude"] = msg.lat / 1e7
        state["longitude"] = msg.lon / 1e7
        relative_alt = getattr(msg, "relative_alt", None)
        alt_mm = relative_alt if relative_alt is not None else msg.alt
        state["altitude"] = alt_mm / 1000.0  # mm -> m
        if msg.hdg != 65535:  # 65535 は不明値
            state["heading"] = int(msg.hdg / 100)  # cdeg -> deg
        return True

    return False


async def receive_loop():
    """MAVLink を継続受信する。executor 経由でイベントループをブロックしない。"""
    loop = asyncio.get_running_loop()
    while vehicle is not None:
        try:
            msg = await loop.run_in_executor(
                None, lambda: vehicle.recv_match(blocking=True, timeout=0.1)
            )
        except Exception:
            logger.exception("MAVLink receive error")
            await asyncio.sleep(0.5)
            continue
        if msg is None:
            continue
        try:
            if handle_mavlink_message(msg):
                await broadcast_state()
        except Exception:
            logger.exception("MAVLink message handling error")


async def ensure_guided():
    """GUIDED でなければ切替を試み、最大 GUIDED_WAIT_SEC 待つ。"""
    if state["mode"] == "GUIDED":
        return True
    vehicle.set_mode("GUIDED")
    deadline = time.time() + GUIDED_WAIT_SEC
    while time.time() < deadline:
        if state["mode"] == "GUIDED":
            return True
        await asyncio.sleep(0.1)
    return False


# ---------------------------------------------------------------- コマンド処理

async def handle_command(cmd):
    """コマンドを実行し、結果メッセージを返す。

    成否は即断せず「送信した」ことだけ伝える。実際の状態変化は
    テレメトリー受信（HEARTBEAT / GLOBAL_POSITION_INT）で反映される。
    """
    ctype = cmd.get("type")

    if ctype == "connect":
        async with connect_lock:
            if state["connected"]:
                return "既に機体へ接続済みです"
            try:
                await connect_vehicle()
            except Exception as exc:
                logger.exception("connect failed")
                return "接続に失敗しました: {}".format(exc)
            await broadcast_state()
            return "機体に接続しました（{}）".format(CONNECTION_STRING)

    if not state["connected"] or vehicle is None:
        return "機体に未接続です。先に「接続」を押してください"

    if ctype == "arm":
        vehicle.mav.command_long_send(
            vehicle.target_system, vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0,
        )
        return "アームコマンドを送信しました"

    if ctype == "disarm":
        vehicle.mav.command_long_send(
            vehicle.target_system, vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0,
        )
        return "ディスアームコマンドを送信しました"

    if ctype == "takeoff":
        try:
            altitude = float(cmd.get("altitude"))
        except (TypeError, ValueError):
            return "離陸高度が不正です"
        if not await ensure_guided():
            return "GUIDED モードへ切り替えられませんでした"
        vehicle.mav.command_long_send(
            vehicle.target_system, vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude,
        )
        return "離陸コマンドを送信しました（目標高度 {} m）".format(altitude)

    if ctype == "land":
        vehicle.mav.command_long_send(
            vehicle.target_system, vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0,
        )
        return "着陸コマンドを送信しました"

    if ctype == "goto":
        try:
            lat = float(cmd.get("latitude"))
            lon = float(cmd.get("longitude"))
            alt = float(cmd.get("altitude"))
        except (TypeError, ValueError):
            return "GoTo の緯度・経度・高度が不正です"
        if not await ensure_guided():
            return "GUIDED モードへ切り替えられませんでした"
        vehicle.mav.set_position_target_global_int_send(
            0,
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,  # 位置のみ有効（速度・加速度・yaw は無効化）
            int(lat * 1e7),
            int(lon * 1e7),
            alt,
            0, 0, 0,
            0, 0, 0,
            0, 0,
        )
        return "GoTo コマンドを送信しました（{:.6f}, {:.6f}, {} m）".format(lat, lon, alt)

    if ctype == "mode":
        mode = str(cmd.get("mode", "")).upper()
        if mode not in ALLOWED_MODES:
            return "未対応のモードです: {}".format(mode)
        try:
            vehicle.set_mode(mode)
        except Exception as exc:
            logger.exception("set_mode failed")
            return "モード変更に失敗しました: {}".format(exc)
        return "モード変更コマンドを送信しました（{}）".format(mode)

    return "不明なコマンドです: {}".format(ctype)


# ---------------------------------------------------------------- WebSocket

async def broadcast_state():
    if not clients:
        return
    message = json.dumps({"type": "state", "state": state})
    for ws in list(clients):
        try:
            await ws.send_text(message)
        except Exception:
            clients.discard(ws)


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    clients.add(ws)
    try:
        # 接続直後に現在の状態を即時送信
        await ws.send_text(json.dumps({"type": "state", "state": state}))
        while True:
            raw = await ws.receive_text()
            try:
                cmd = json.loads(raw)
            except json.JSONDecodeError:
                await ws.send_text(json.dumps(
                    {"type": "status", "message": "不正な JSON を受信しました"}
                ))
                continue
            message = await handle_command(cmd)
            await ws.send_text(json.dumps({"type": "status", "message": message}))
    except WebSocketDisconnect:
        pass
    except Exception:
        logger.exception("WebSocket error")
    finally:
        clients.discard(ws)


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="127.0.0.1", port=9999)
