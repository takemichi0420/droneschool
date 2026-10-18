from __future__ import annotations

import asyncio
import json
import logging
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any, Dict, List, Optional

import os
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("drone_web_app")

BASE_DIR = Path(__file__).resolve().parent.parent
FRONTEND_DIR = BASE_DIR / "frontend"

app = FastAPI(title="Drone Web Controller")
app.mount("/static", StaticFiles(directory=str(FRONTEND_DIR)), name="static")

state_lock = threading.Lock()
state: Dict[str, Any] = {
    "connected": False,
    "armed": False,
    "mode": "UNKNOWN",
    "latitude": 0.0,
    "longitude": 0.0,
    "altitude": 0.0,
    "heading": 0,
}

mav_connection: Optional[Any] = None
mav_thread: Optional[threading.Thread] = None
stop_event = threading.Event()
connection_lock = threading.Lock()
main_loop: Optional[asyncio.AbstractEventLoop] = None
executor = ThreadPoolExecutor(max_workers=1)

# Connection string can be overridden for local testing
connection_string = os.environ.get("MAV_ENDPOINT", "udpout:host.docker.internal:14550")
MODE_MAP: Dict[int, str] = {}


class ConnectionManager:
    def __init__(self) -> None:
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket) -> None:
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket) -> None:
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    async def broadcast(self, message: str) -> None:
        stale: List[WebSocket] = []
        for connection in self.active_connections:
            try:
                await connection.send_text(message)
            except Exception:
                stale.append(connection)
        for connection in stale:
            self.disconnect(connection)


manager = ConnectionManager()


def get_state_snapshot() -> Dict[str, Any]:
    with state_lock:
        return {
            "connected": bool(state["connected"]),
            "armed": bool(state["armed"]),
            "mode": state["mode"],
            "latitude": round(float(state["latitude"]), 6),
            "longitude": round(float(state["longitude"]), 6),
            "altitude": round(float(state["altitude"]), 2),
            "heading": int(state["heading"]),
        }


def set_state_value(key: str, value: Any) -> None:
    with state_lock:
        state[key] = value


def notify_state() -> None:
    if main_loop is None or main_loop.is_closed():
        return
    payload = {"type": "state", "state": get_state_snapshot()}
    main_loop.call_soon_threadsafe(lambda: asyncio.create_task(manager.broadcast(json.dumps(payload))))


def notify_status(message: str) -> None:
    if main_loop is None or main_loop.is_closed():
        return
    payload = {"type": "status", "message": message}
    main_loop.call_soon_threadsafe(lambda: asyncio.create_task(manager.broadcast(json.dumps(payload))))


def send_mav_command(command: str, payload: Optional[Dict[str, Any]] = None) -> None:
    global mav_connection
    global hover_target_altitude
    global hover_target_sent
    if mav_connection is None:
        notify_status("Vehicle is not connected")
        return
    try:
        if command == "arm":
            mav_connection.mav.command_long_send(
                mav_connection.target_system,
                mav_connection.target_component,
                mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1,
                0,
                0,
                0,
                0,
                0,
                0,
            )
        elif command == "disarm":
            mav_connection.mav.command_long_send(
                mav_connection.target_system,
                mav_connection.target_component,
                mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            )
        elif command == "takeoff":
            altitude = float(payload.get("altitude", 0)) if payload else 0.0
            notify_status(f"Takeoff request: {altitude:.2f} m")
            mav_connection.mav.command_long_send(
                mav_connection.target_system,
                mav_connection.target_component,
                mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                altitude,
            )
        elif command == "land":
            mav_connection.mav.command_long_send(
                mav_connection.target_system,
                mav_connection.target_component,
                mavlink.MAV_CMD_NAV_LAND,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            )
        elif command == "goto":
            latitude = float(payload.get("latitude", 0)) if payload else 0.0
            longitude = float(payload.get("longitude", 0)) if payload else 0.0
            altitude = float(payload.get("altitude", 0)) if payload else 0.0
            mav_connection.mav.set_position_target_global_int_send(
                int(time.time() * 1000) & 0xFFFFFFFF,
                mav_connection.target_system,
                mav_connection.target_component,
                mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                0b0000111111111000,
                int(latitude * 1e7),
                int(longitude * 1e7),
                altitude,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            )
        elif command == "mode":
            mode_name = payload.get("mode") if payload else None
            if mode_name:
                mav_connection.set_mode(mode_name)
    except Exception as exc:  # pragma: no cover - defensive logging
        logger.exception("Command send failed: %s", exc)
        notify_status(f"Command failed: {exc}")


def ensure_guided(timeout: float = 5.0) -> bool:
    if mav_connection is None:
        return False
    if state.get("mode") == "GUIDED":
        return True
    notify_status("Switching to GUIDED mode...")
    send_mav_command("mode", {"mode": "GUIDED"})
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if get_state_snapshot()["mode"] == "GUIDED":
            return True
        time.sleep(0.2)
    logger.warning("GUIDED mode switch timed out")
    return False


def run_vehicle_loop() -> None:
    global mav_connection
    global mav_thread
    try:
        stop_event.clear()
        notify_status(f"Connecting to vehicle at {connection_string}...")
        # Establish connection and detect autopilot heartbeat without blocking import/startup
        with connection_lock:
            m = mavutil.mavlink_connection(connection_string)
        logger.info("Connecting to MAVLink at %s", connection_string)

        # send GCS heartbeat to stimulate responses if needed
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                             mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

        hb = None
        deadline = time.time() + 30
        while time.time() < deadline:
            try:
                msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            except Exception:
                msg = None
            if msg and getattr(msg, "autopilot", None) != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                hb = msg
                break

        if hb is None:
            raise RuntimeError("no autopilot heartbeat detected")

        # set target system/component and mode map based on heartbeat
        m.target_system = hb.get_srcSystem()
        m.target_component = hb.get_srcComponent()
        global MODE_MAP
        MODE_MAP = mavutil.mode_mapping_byname(getattr(hb, "type", None)) or {}

        with connection_lock:
            mav_connection = m

        set_state_value("connected", True)
        notify_state()
        notify_status("Vehicle connected")

        mav_connection.mav.request_data_stream_send(
            mav_connection.target_system,
            mav_connection.target_component,
            mavlink.MAV_DATA_STREAM_ALL,
            4,
            1,
        )

        while not stop_event.is_set():
            msg = mav_connection.recv_match(blocking=True, timeout=0.1)
            if msg is None:
                continue
            msg_type = msg.get_type()
            if msg_type == "GLOBAL_POSITION_INT":
                # only process messages from the connected vehicle
                if getattr(msg, "get_srcSystem", lambda: None)() != mav_connection.target_system:
                    continue
                with state_lock:
                    state["latitude"] = msg.lat / 1e7
                    state["longitude"] = msg.lon / 1e7
                    if msg.relative_alt is not None:
                        state["altitude"] = msg.relative_alt / 1000.0
                    else:
                        state["altitude"] = msg.alt / 1000.0
                    if msg.hdg != 65535:
                        state["heading"] = int(msg.hdg / 100.0)
                notify_state()
            elif msg_type == "HEARTBEAT":
                if getattr(msg, "type", None) == mavlink.MAV_TYPE_GCS:
                    continue
                if getattr(msg, "autopilot", None) == mavlink.MAV_AUTOPILOT_INVALID:
                    continue
                # filter to only vehicle of interest
                if getattr(msg, "get_srcSystem", lambda: None)() != mav_connection.target_system:
                    continue
                if getattr(msg, "get_srcComponent", lambda: None)() != mav_connection.target_component:
                    continue
                with state_lock:
                    state["armed"] = bool(msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    mode_mapping = {value: name for name, value in MODE_MAP.items()} if MODE_MAP else {value: name for name, value in mav_connection.mode_mapping().items()}
                    state["mode"] = mode_mapping.get(msg.custom_mode, str(msg.custom_mode))
                notify_state()
    except Exception as exc:  # pragma: no cover - defensive logging
        logger.exception("MAVLink loop failed: %s", exc)
        notify_status(f"Vehicle connection error: {exc}")
    finally:
        with state_lock:
            state["connected"] = False
        notify_state()
        with connection_lock:
            mav_connection = None


@app.on_event("startup")
async def startup_event() -> None:
    global main_loop
    main_loop = asyncio.get_running_loop()


@app.get("/")
async def index() -> FileResponse:
    return FileResponse(FRONTEND_DIR / "index.html")


@app.get("/register_service")
async def register_service():
    return {
        "name": "Drone Web Controller",
        "description": "FastAPI-based drone web controller (WebSocket realtime)",
        "icon": "mdi-drone",
        "company": "",
        "version": "1.0.0",
        "webpage": "",
        "api": "/docs",
        "avoid_iframes": True,
    }


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket) -> None:
    await manager.connect(websocket)
    try:
        await websocket.send_text(json.dumps({"type": "state", "state": get_state_snapshot()}))
        while True:
            raw = await websocket.receive_text()
            try:
                payload = json.loads(raw)
            except json.JSONDecodeError:
                await websocket.send_text(json.dumps({"type": "status", "message": "Invalid JSON"}))
                continue

            command_type = payload.get("type")
            if command_type == "connect":
                if mav_connection is not None and getattr(mav_connection, "target_system", None) is not None:
                    await websocket.send_text(json.dumps({"type": "status", "message": "Already connected"}))
                    continue
                global mav_thread
                if mav_thread is None or not mav_thread.is_alive():
                    mav_thread = threading.Thread(target=run_vehicle_loop, daemon=True)
                    mav_thread.start()
                await websocket.send_text(json.dumps({"type": "status", "message": "Connection attempt started"}))
                continue

            if command_type == "arm":
                send_mav_command("arm")
                await websocket.send_text(json.dumps({"type": "status", "message": "Arm command sent"}))
            elif command_type == "disarm":
                send_mav_command("disarm")
                await websocket.send_text(json.dumps({"type": "status", "message": "Disarm command sent"}))
            elif command_type == "takeoff":
                if ensure_guided():
                    send_mav_command("takeoff", {"altitude": payload.get("altitude", 0)})
                    await websocket.send_text(json.dumps({"type": "status", "message": "Takeoff command sent"}))
                else:
                    await websocket.send_text(json.dumps({"type": "status", "message": "GUIDED mode switch timed out"}))
            elif command_type == "land":
                send_mav_command("land")
                await websocket.send_text(json.dumps({"type": "status", "message": "Land command sent"}))
            elif command_type == "goto":
                if ensure_guided():
                    send_mav_command("goto", payload)
                    await websocket.send_text(json.dumps({"type": "status", "message": "Goto command sent"}))
                else:
                    await websocket.send_text(json.dumps({"type": "status", "message": "GUIDED mode switch timed out"}))
            elif command_type == "mode":
                send_mav_command("mode", payload)
                await websocket.send_text(json.dumps({"type": "status", "message": "Mode command sent"}))
            else:
                await websocket.send_text(json.dumps({"type": "status", "message": "Unknown command"}))
    except WebSocketDisconnect:
        manager.disconnect(websocket)
    except Exception as exc:  # pragma: no cover - defensive logging
        logger.exception("WebSocket exception: %s", exc)
        manager.disconnect(websocket)
