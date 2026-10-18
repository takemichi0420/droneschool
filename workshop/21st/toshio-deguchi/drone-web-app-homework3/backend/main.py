import asyncio
import json
import logging
import os
import time
from typing import Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pymavlink import mavutil

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("drone-web-app-homework3")

CONNECTION_STRING = os.environ.get("MAV_ENDPOINT", "udpout:host.docker.internal:14550")
ALLOWED_MODES = {"GUIDED", "AUTO", "RTL", "LOITER", "STABILIZE"}
GUIDED_SWITCH_TIMEOUT = 5.0
LIFTOFF_ALTITUDE_THRESHOLD = 0.5
LIFTOFF_TIMEOUT = 15.0
AUTOPILOT_HEARTBEAT_TIMEOUT = 30.0

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
FRONTEND_DIR = os.path.normpath(os.path.join(BASE_DIR, "..", "frontend"))

app = FastAPI()
app.mount("/static", StaticFiles(directory=FRONTEND_DIR), name="static")

state = {
    "connected": False,
    "armed": False,
    "mode": "UNKNOWN",
    "latitude": 0.0,
    "longitude": 0.0,
    "altitude": 0.0,
    "heading": 0,
    "battery": 0,
}


class ConnectionManager:
    def __init__(self):
        self.active: list[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active.append(websocket)

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active:
            self.active.remove(websocket)

    async def broadcast(self, message: dict):
        dead = []
        for websocket in self.active:
            try:
                await websocket.send_json(message)
            except Exception:
                dead.append(websocket)
        for websocket in dead:
            self.disconnect(websocket)


manager = ConnectionManager()


class DroneController:
    def __init__(self):
        self.vehicle: Optional[mavutil.mavfile] = None
        self.reader_task: Optional[asyncio.Task] = None
        self.connecting = False
        self.takeoff_pending = False
        self.mode_mapping: dict = {}
        self.reverse_mode_mapping: dict = {}

    async def connect(self) -> str:
        if self.vehicle is not None:
            return "既に接続済みです"
        if self.connecting:
            return "接続処理中です"

        self.connecting = True
        loop = asyncio.get_event_loop()
        try:
            vehicle, mode_map = await loop.run_in_executor(None, self._connect_sync)
        except Exception as e:
            self.connecting = False
            logger.exception("MAVLink接続に失敗しました")
            return f"接続に失敗しました: {e}"

        self.vehicle = vehicle
        self.mode_mapping = mode_map
        self.reverse_mode_mapping = {v: k for k, v in mode_map.items()}
        self.connecting = False
        state["connected"] = True

        vehicle.mav.request_data_stream_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            4,
            1,
        )
        vehicle.mav.request_data_stream_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
            2,
            1,
        )

        self.reader_task = asyncio.create_task(self._read_loop())
        await manager.broadcast({"type": "state", "state": state})
        return "MAVLink接続が完了しました"

    def _connect_sync(self):
        vehicle = mavutil.mavlink_connection(CONNECTION_STRING)
        # mavlink-router 経由では GCS 自身の HEARTBEAT も送り返されてくるため、
        # autopilot（非GCS）の HEARTBEAT だけを待って機体を特定する。
        vehicle.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0,
        )
        deadline = time.monotonic() + AUTOPILOT_HEARTBEAT_TIMEOUT
        autopilot_hb = None
        while time.monotonic() < deadline:
            msg = vehicle.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if msg is not None and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                autopilot_hb = msg
                break
        if autopilot_hb is None:
            vehicle.close()
            raise TimeoutError("autopilotからのHEARTBEATを受信できませんでした")

        vehicle.target_system = autopilot_hb.get_srcSystem()
        vehicle.target_component = autopilot_hb.get_srcComponent()
        # mode_mapping() は直近に受信したHEARTBEATの機体種別を見るため、GCSのHEARTBEATが
        # 混在すると誤ったマップを返しうる。特定済みのautopilotの種別から明示的に生成する。
        mode_map = mavutil.mode_mapping_byname(autopilot_hb.type) or {}
        return vehicle, mode_map

    async def _read_loop(self):
        loop = asyncio.get_event_loop()
        vehicle = self.vehicle
        try:
            while True:
                msg = await loop.run_in_executor(
                    None, lambda: vehicle.recv_match(blocking=True, timeout=0.1)
                )
                if msg is None:
                    continue
                if self._handle_message(msg):
                    await manager.broadcast({"type": "state", "state": state})
        except asyncio.CancelledError:
            raise
        except Exception:
            logger.exception("MAVLink受信中にエラーが発生しました")

    def _handle_message(self, msg) -> bool:
        vehicle = self.vehicle
        # mavlink-router 経由では他機体やGCSのメッセージも流れてくるため、
        # 接続時に特定した自機（target_system/component）以外は無視する。
        # これによりHEARTBEATの点滅やモード誤認を防ぐ。
        if (
            msg.get_srcSystem() != vehicle.target_system
            or msg.get_srcComponent() != vehicle.target_component
        ):
            return False

        msg_type = msg.get_type()

        if msg_type == "GLOBAL_POSITION_INT":
            state["latitude"] = round(msg.lat / 1e7, 6)
            state["longitude"] = round(msg.lon / 1e7, 6)
            alt_mm = getattr(msg, "relative_alt", None)
            if alt_mm is None:
                alt_mm = msg.alt
            state["altitude"] = round(alt_mm / 1000, 2)
            if msg.hdg != 65535:
                state["heading"] = int(msg.hdg / 100)
            return True

        if msg_type == "SYS_STATUS":
            if msg.battery_remaining != -1:
                state["battery"] = int(msg.battery_remaining)
            return True

        if msg_type == "HEARTBEAT":
            state["mode"] = self.reverse_mode_mapping.get(msg.custom_mode, state["mode"])
            state["armed"] = bool(
                msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )
            return True

        return False

    def _ensure_connected(self):
        if self.vehicle is None:
            raise RuntimeError("機体が未接続です")

    def _send_set_mode(self, mode_name: str) -> bool:
        # vehicle.set_mode(str) は pymavlink 内部の mode_mapping() に依存しており、
        # GCSのHEARTBEATが混在するとその時点の機体種別を誤って参照しうる。
        # 接続時に特定した自機のモードマップ（self.mode_mapping）を明示的に使う。
        mode_number = self.mode_mapping.get(mode_name)
        if mode_number is None:
            return False
        vehicle = self.vehicle
        vehicle.mav.set_mode_send(
            vehicle.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_number,
        )
        return True

    async def _switch_to_guided(self) -> bool:
        if state["mode"] == "GUIDED":
            return True
        self._send_set_mode("GUIDED")
        start = time.monotonic()
        while time.monotonic() - start < GUIDED_SWITCH_TIMEOUT:
            if state["mode"] == "GUIDED":
                return True
            await asyncio.sleep(0.1)
        return state["mode"] == "GUIDED"

    async def _wait_for_liftoff(self) -> bool:
        if not self.takeoff_pending:
            return True
        start = time.monotonic()
        while time.monotonic() - start < LIFTOFF_TIMEOUT:
            if not self.takeoff_pending:
                return True
            await asyncio.sleep(0.1)
        return not self.takeoff_pending

    async def _monitor_liftoff(self):
        start = time.monotonic()
        while time.monotonic() - start < LIFTOFF_TIMEOUT:
            if state["altitude"] >= LIFTOFF_ALTITUDE_THRESHOLD:
                break
            await asyncio.sleep(0.1)
        self.takeoff_pending = False

    async def arm(self) -> str:
        self._ensure_connected()
        vehicle = self.vehicle
        vehicle.mav.command_long_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0,
        )
        return "アームコマンドを送信しました"

    async def disarm(self) -> str:
        self._ensure_connected()
        vehicle = self.vehicle
        vehicle.mav.command_long_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0,
        )
        return "ディスアームコマンドを送信しました"

    async def takeoff(self, altitude: float) -> str:
        self._ensure_connected()
        if not await self._switch_to_guided():
            return "GUIDEDモードへの切替がタイムアウトしたため、離陸コマンドは送信していません"
        vehicle = self.vehicle
        self.takeoff_pending = True
        vehicle.mav.command_long_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0, altitude,
        )
        asyncio.create_task(self._monitor_liftoff())
        return f"離陸コマンドを送信しました（目標高度 {altitude}m）"

    async def land(self) -> str:
        self._ensure_connected()
        vehicle = self.vehicle
        vehicle.mav.command_long_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0, 0, 0, 0,
        )
        return "着陸コマンドを送信しました"

    async def goto(self, latitude: float, longitude: float, altitude: float) -> str:
        self._ensure_connected()
        if not await self._switch_to_guided():
            return "GUIDEDモードへの切替がタイムアウトしたため、GoToコマンドは送信していません"
        if not await self._wait_for_liftoff():
            return "離陸が完了していないため、GoToコマンドは送信していません"
        vehicle = self.vehicle
        type_mask = 0b0000111111111000
        vehicle.mav.set_position_target_global_int_send(
            0,
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            type_mask,
            int(latitude * 1e7),
            int(longitude * 1e7),
            altitude,
            0, 0, 0,
            0, 0, 0,
            0, 0,
        )
        return f"GoToコマンドを送信しました（{latitude}, {longitude}, {altitude}m）"

    async def set_mode(self, mode: str) -> str:
        self._ensure_connected()
        if mode not in ALLOWED_MODES:
            return f"未対応のモードです: {mode}"
        if not self._send_set_mode(mode):
            return f"機体がサポートしていないモードです: {mode}"
        return f"モード変更コマンドを送信しました: {mode}"


controller = DroneController()


async def handle_command(msg: dict) -> str:
    msg_type = msg.get("type")
    try:
        if msg_type == "connect":
            return await controller.connect()
        if msg_type == "arm":
            return await controller.arm()
        if msg_type == "disarm":
            return await controller.disarm()
        if msg_type == "takeoff":
            return await controller.takeoff(float(msg.get("altitude", 0)))
        if msg_type == "land":
            return await controller.land()
        if msg_type == "goto":
            return await controller.goto(
                float(msg["latitude"]), float(msg["longitude"]), float(msg["altitude"])
            )
        if msg_type == "mode":
            return await controller.set_mode(str(msg.get("mode", "")))
        return f"未対応のコマンドです: {msg_type}"
    except RuntimeError as e:
        return str(e)
    except (KeyError, ValueError, TypeError) as e:
        return f"不正なパラメータです: {e}"
    except Exception:
        logger.exception("コマンド処理中にエラーが発生しました: %s", msg_type)
        return "コマンド処理中にエラーが発生しました"


@app.get("/")
async def index():
    return FileResponse(os.path.join(FRONTEND_DIR, "index.html"))


@app.get("/register_service")
async def register_service():
    return {
        "name": "Drone Web App HW3",
        "description": "ブラウザからMAVLink機体（Copter）を操作するドローンWeb制御アプリ",
        "icon": "mdi-quadcopter",
        "company": "",
        "version": "1.0.0",
        "webpage": "",
        "api": "/docs",
        # WebSocketはBlueOSのHTTPプロキシ経由では動作しないため、
        # 左メニューから直接 http://<IP>:<port>/ を新規ウィンドウで開く。
        "avoid_iframes": True,
    }


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    await websocket.send_json({"type": "state", "state": state})
    try:
        while True:
            data = await websocket.receive_text()
            try:
                msg = json.loads(data)
            except json.JSONDecodeError:
                await websocket.send_json(
                    {"type": "status", "message": "不正なJSON形式です"}
                )
                continue
            result = await handle_command(msg)
            await websocket.send_json({"type": "status", "message": result})
    except WebSocketDisconnect:
        manager.disconnect(websocket)
    except Exception:
        logger.exception("WebSocket処理中にエラーが発生しました")
        manager.disconnect(websocket)


@app.on_event("shutdown")
async def shutdown_event():
    if controller.reader_task is not None:
        controller.reader_task.cancel()
    if controller.vehicle is not None:
        controller.vehicle.close()
