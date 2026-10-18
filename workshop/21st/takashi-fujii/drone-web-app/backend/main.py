import asyncio
import json
import logging
import os
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pymavlink import mavutil

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

APP_NAME = "Drone Web Control App"
CONNECTION_STRING = os.environ.get("MAV_ENDPOINT", "udpout:host.docker.internal:14550")
BASE_DIR = Path(__file__).resolve().parent
FRONTEND_DIR = BASE_DIR.parent / "frontend"
INDEX_FILE = FRONTEND_DIR / "index.html"

app = FastAPI()
app.mount("/static", StaticFiles(directory=str(FRONTEND_DIR)), name="static")

drone_state = {
    "connected": False,
    "armed": False,
    "mode": "UNKNOWN",
    "latitude": 0.0,
    "longitude": 0.0,
    "altitude": 0.0,
    "heading": 0,
}

vehicle = None
connected_clients = set()
executor = ThreadPoolExecutor(max_workers=4)
event_loop = None
connect_lock = threading.Lock()
receiver_started = False
connecting = False
MODE_MAP = {}
REVERSE_MODE_MAP = {}


@app.get("/")
async def get_index():
    return FileResponse(INDEX_FILE)


@app.get("/register_service")
async def register_service():
    return {
        "name": APP_NAME,
        "description": "BlueOS extension for MAVLink-based drone control and telemetry.",
        "icon": "mdi-drone",
        "company": "",
        "version": "1.0.0",
        "webpage": "",
        "api": "/docs",
        "avoid_iframes": True,
    }


def _set_connection_state(connected: bool):
    drone_state["connected"] = connected
    if not connected:
        drone_state["armed"] = False
        drone_state["mode"] = "UNKNOWN"


async def broadcast_state():
    if not connected_clients:
        return

    payload = json.dumps({"type": "state", "state": drone_state})
    stale_clients = []
    for client in list(connected_clients):
        try:
            await client.send_text(payload)
        except Exception:
            stale_clients.append(client)

    for client in stale_clients:
        connected_clients.discard(client)


def _schedule_broadcast():
    if event_loop is None or event_loop.is_closed():
        return
    asyncio.run_coroutine_threadsafe(broadcast_state(), event_loop)


def _close_vehicle():
    global vehicle
    if vehicle is None:
        return
    try:
        vehicle.close()
    except Exception:
        pass
    vehicle = None


def _connect():
    global vehicle, connecting, MODE_MAP, REVERSE_MODE_MAP

    try:
        while vehicle is None:
            try:
                logger.info("Connecting to %s", CONNECTION_STRING)
                mav = mavutil.mavlink_connection(CONNECTION_STRING)
                try:
                    mav.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0,
                        0,
                        0,
                    )
                except Exception:
                    logger.debug("Failed to send probe heartbeat", exc_info=True)

                heartbeat = None
                deadline = time.time() + 30
                while time.time() < deadline:
                    msg = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
                    if msg and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                        heartbeat = msg
                        break

                if heartbeat is None:
                    logger.warning("No autopilot heartbeat found, retrying")
                    try:
                        mav.close()
                    except Exception:
                        pass
                    time.sleep(3)
                    continue

                mav.target_system = heartbeat.get_srcSystem()
                mav.target_component = heartbeat.get_srcComponent()
                MODE_MAP = mavutil.mode_mapping_byname(heartbeat.type) or {}
                REVERSE_MODE_MAP = {value: key for key, value in MODE_MAP.items()}
                mav.mav.request_data_stream_send(
                    mav.target_system,
                    mav.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_ALL,
                    4,
                    1,
                )
                vehicle = mav
                _set_connection_state(True)
                _schedule_broadcast()
                logger.info(
                    "Connected to vehicle sys=%s comp=%s",
                    mav.target_system,
                    mav.target_component,
                )
            except Exception as exc:
                logger.warning("connect retry: %s", exc)
                time.sleep(3)
    finally:
        connecting = False


def ensure_vehicle_connection():
    global connecting
    with connect_lock:
        if vehicle is not None or connecting:
            return False
        connecting = True
        threading.Thread(target=_connect, daemon=True).start()
        return True


def _target_matches(message):
    if vehicle is None:
        return False
    try:
        return (
            message.get_srcSystem() == vehicle.target_system
            and message.get_srcComponent() == vehicle.target_component
        )
    except Exception:
        return False


def mavlink_receiver():
    global vehicle

    while True:
        try:
            if vehicle is None:
                time.sleep(0.1)
                continue

            msg = vehicle.recv_match(blocking=True, timeout=0.2)
            if not msg or not _target_matches(msg):
                continue

            msg_type = msg.get_type()
            updated = False

            if msg_type == "HEARTBEAT":
                armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                if drone_state["armed"] != armed:
                    drone_state["armed"] = armed
                    updated = True

                mode_name = REVERSE_MODE_MAP.get(msg.custom_mode, mavutil.mode_string_v10(msg))
                if mode_name and drone_state["mode"] != mode_name:
                    drone_state["mode"] = mode_name
                    updated = True

            elif msg_type == "GLOBAL_POSITION_INT":
                latitude = msg.lat / 1e7
                longitude = msg.lon / 1e7
                altitude = getattr(msg, "relative_alt", msg.alt) / 1000.0
                heading = 0.0 if msg.hdg == 65535 else msg.hdg / 100.0

                if drone_state["latitude"] != latitude:
                    drone_state["latitude"] = latitude
                    updated = True
                if drone_state["longitude"] != longitude:
                    drone_state["longitude"] = longitude
                    updated = True
                if drone_state["altitude"] != altitude:
                    drone_state["altitude"] = altitude
                    updated = True
                if drone_state["heading"] != heading:
                    drone_state["heading"] = heading
                    updated = True

            if updated:
                _schedule_broadcast()
        except Exception as exc:
            logger.error("MAVLink receive error: %s", exc)
            _close_vehicle()
            _set_connection_state(False)
            _schedule_broadcast()
            time.sleep(1)


def change_mode_sync(mode_name: str):
    if vehicle is None or not drone_state["connected"]:
        return False

    mode_id = MODE_MAP.get(mode_name.upper())
    if mode_id is None:
        logger.error("Unknown mode: %s", mode_name)
        return False

    try:
        vehicle.set_mode(mode_id)
        return True
    except Exception as exc:
        logger.error("Failed to change mode: %s", exc)
        return False


@app.on_event("startup")
async def startup_event():
    global event_loop, receiver_started
    event_loop = asyncio.get_running_loop()
    if not receiver_started:
        receiver_started = True
        threading.Thread(target=mavlink_receiver, daemon=True).start()


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    connected_clients.add(websocket)
    await websocket.send_text(json.dumps({"type": "state", "state": drone_state}))

    try:
        while True:
            data = await websocket.receive_text()
            cmd = json.loads(data)
            cmd_type = cmd.get("type")

            if cmd_type == "connect":
                started = ensure_vehicle_connection()
                message = "Connecting to vehicle" if started else "Already connected or connecting"
                await websocket.send_text(json.dumps({"type": "status", "message": message}))

            elif cmd_type == "arm" and vehicle is not None:
                vehicle.mav.command_long_send(
                    vehicle.target_system,
                    vehicle.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    1,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
                await websocket.send_text(json.dumps({"type": "status", "message": "Arm command sent"}))

            elif cmd_type == "disarm" and vehicle is not None:
                vehicle.mav.command_long_send(
                    vehicle.target_system,
                    vehicle.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
                await websocket.send_text(json.dumps({"type": "status", "message": "Disarm command sent"}))

            elif cmd_type == "takeoff" and vehicle is not None:
                loop = asyncio.get_running_loop()
                await loop.run_in_executor(executor, change_mode_sync, "GUIDED")
                await asyncio.sleep(1)
                altitude = float(cmd.get("altitude", 10))
                vehicle.mav.command_long_send(
                    vehicle.target_system,
                    vehicle.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    altitude,
                )
                await websocket.send_text(
                    json.dumps({"type": "status", "message": f"Takeoff command sent (Alt: {altitude}m)"})
                )

            elif cmd_type == "land" and vehicle is not None:
                vehicle.mav.command_long_send(
                    vehicle.target_system,
                    vehicle.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_LAND,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
                await websocket.send_text(json.dumps({"type": "status", "message": "Land command sent"}))

            elif cmd_type == "goto" and vehicle is not None:
                loop = asyncio.get_running_loop()
                await loop.run_in_executor(executor, change_mode_sync, "GUIDED")
                await asyncio.sleep(1)
                latitude = float(cmd.get("latitude", 0))
                longitude = float(cmd.get("longitude", 0))
                altitude = float(cmd.get("altitude", 0))

                vehicle.mav.set_position_target_global_int_send(
                    0,
                    vehicle.target_system,
                    vehicle.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
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
                await websocket.send_text(
                    json.dumps(
                        {
                            "type": "status",
                            "message": f"GoTo command sent ({latitude}, {longitude}, {altitude}m)",
                        }
                    )
                )

            elif cmd_type == "mode" and vehicle is not None:
                mode_name = cmd.get("mode", "")
                loop = asyncio.get_running_loop()
                success = await loop.run_in_executor(executor, change_mode_sync, mode_name)
                message = f"Mode change to {mode_name} sent" if success else "Mode change failed"
                await websocket.send_text(json.dumps({"type": "status", "message": message}))

    except WebSocketDisconnect:
        pass
    finally:
        connected_clients.discard(websocket)
