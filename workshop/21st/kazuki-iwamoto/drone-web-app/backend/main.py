import asyncio
import json
import logging
import os
import time
from pathlib import Path
from typing import Any, Dict, Optional, Set

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pymavlink import mavutil


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("drone-web-app")

BASE_DIR = Path(__file__).resolve().parent.parent
FRONTEND_DIR = BASE_DIR / "frontend"
DEFAULT_CONNECTION = os.environ.get(
    "MAV_ENDPOINT", "udpout:host.docker.internal:14550"
)
GUIDED_WAIT_SECONDS = 5.0
POSITION_TYPE_MASK = 0b0000111111111000

STATE: Dict[str, Any] = {
    "connected": False,
    "armed": False,
    "mode": "UNKNOWN",
    "latitude": 0.0,
    "longitude": 0.0,
    "altitude": 0.0,
    "heading": 0,
}


class DroneController:
    def __init__(self, connection_string: str = DEFAULT_CONNECTION) -> None:
        self.connection_string = connection_string
        self.vehicle = None
        self.connected = False
        self.connecting = False
        self.target_system = 1
        self.target_component = 1
        self.mode_map: Dict[str, int] = {}
        self._receiver_task: Optional[asyncio.Task] = None
        self._lock = asyncio.Lock()

    async def connect(self) -> str:
        async with self._lock:
            if self.connected:
                return "Already connected to vehicle."
            if self.connecting:
                return "Connection is already in progress."

            self.connecting = True
            try:
                loop = asyncio.get_running_loop()
                vehicle = await loop.run_in_executor(None, self._connect_blocking)
                self.vehicle = vehicle
                self.connected = True
                STATE["connected"] = True
                self._request_position_stream()
                self._start_receiver()
                await broadcast_state()
                return f"Connected to vehicle via {self.connection_string}."
            except Exception as exc:
                self.vehicle = None
                self.connected = False
                STATE["connected"] = False
                logger.exception("MAVLink connection failed")
                await broadcast_state()
                return f"Connection failed: {exc}"
            finally:
                self.connecting = False

    def _connect_blocking(self):
        vehicle = mavutil.mavlink_connection(self.connection_string)
        vehicle.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
        )
        deadline = time.time() + 30
        heartbeat = None
        while time.time() < deadline:
            msg = vehicle.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if msg and self._is_autopilot_heartbeat(msg):
                heartbeat = msg
                break
        if heartbeat is None:
            raise TimeoutError("Timed out waiting for autopilot HEARTBEAT.")
        self.target_system = heartbeat.get_srcSystem()
        self.target_component = heartbeat.get_srcComponent()
        vehicle.target_system = self.target_system
        vehicle.target_component = self.target_component
        self.mode_map = mavutil.mode_mapping_byname(heartbeat.type) or {}
        return vehicle

    def _request_position_stream(self) -> None:
        if not self.vehicle:
            return
        try:
            self.vehicle.mav.request_data_stream_send(
                self.vehicle.target_system,
                self.vehicle.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                4,
                1,
            )
        except Exception:
            logger.exception("Failed to request MAVLink data stream")

    def _start_receiver(self) -> None:
        if self._receiver_task and not self._receiver_task.done():
            return
        self._receiver_task = asyncio.create_task(self.receive_loop())

    async def receive_loop(self) -> None:
        while self.connected and self.vehicle:
            try:
                loop = asyncio.get_running_loop()
                msg = await loop.run_in_executor(
                    None,
                    lambda: self.vehicle.recv_match(blocking=True, timeout=0.1),
                )
                if msg is None:
                    continue
                if self._handle_message(msg):
                    await broadcast_state()
            except asyncio.CancelledError:
                raise
            except Exception:
                logger.exception("MAVLink receive error")
                await asyncio.sleep(1)

    def _handle_message(self, msg) -> bool:
        if not self._is_target_message(msg):
            return False

        msg_type = msg.get_type()
        if msg_type == "GLOBAL_POSITION_INT":
            STATE["latitude"] = msg.lat / 1e7
            STATE["longitude"] = msg.lon / 1e7
            relative_alt = getattr(msg, "relative_alt", None)
            altitude_mm = relative_alt if relative_alt is not None else msg.alt
            STATE["altitude"] = altitude_mm / 1000
            hdg = getattr(msg, "hdg", 65535)
            if hdg != 65535:
                STATE["heading"] = int(round(hdg / 100))
            return True

        if msg_type == "HEARTBEAT":
            STATE["armed"] = bool(
                msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )
            STATE["mode"] = self._mode_name_from_heartbeat(msg)
            return True

        return False

    def _is_target_message(self, msg) -> bool:
        return (
            msg.get_srcSystem() == self.target_system
            and msg.get_srcComponent() == self.target_component
        )

    def _is_autopilot_heartbeat(self, msg) -> bool:
        if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
            return False
        if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            return False
        return True

    def _mode_name_from_heartbeat(self, msg) -> str:
        if not self.vehicle:
            return "UNKNOWN"
        try:
            reverse_mapping = {value: key for key, value in self.mode_map.items()}
            return reverse_mapping.get(msg.custom_mode, "UNKNOWN")
        except Exception:
            logger.exception("Failed to resolve flight mode")
            return "UNKNOWN"

    async def command(self, payload: Dict[str, Any]) -> str:
        command_type = payload.get("type")
        if command_type == "connect":
            return await self.connect()
        if command_type == "mode":
            return await self.set_mode(str(payload.get("mode", "")))

        if not self.vehicle or not self.connected:
            return "Vehicle is not connected."

        try:
            if command_type == "arm":
                self._send_arm(True)
                return "Arm command sent."
            if command_type == "disarm":
                self._send_arm(False)
                return "Disarm command sent."
            if command_type == "takeoff":
                altitude = float(payload.get("altitude", 10))
                guided = await self.ensure_guided()
                if not guided:
                    return "Unable to switch to GUIDED mode for takeoff."
                self.vehicle.mav.command_long_send(
                    self.target_system,
                    self.target_component,
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
                return f"Takeoff command sent: {altitude:.1f} m."
            if command_type == "land":
                self.vehicle.mav.command_long_send(
                    self.target_system,
                    self.target_component,
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
                return "Land command sent."
            if command_type == "goto":
                latitude = float(payload["latitude"])
                longitude = float(payload["longitude"])
                altitude = float(payload["altitude"])
                guided = await self.ensure_guided()
                if not guided:
                    return "Unable to switch to GUIDED mode for GoTo."
                self.vehicle.mav.set_position_target_global_int_send(
                    0,
                    self.target_system,
                    self.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    POSITION_TYPE_MASK,
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
                return f"GoTo command sent: {latitude:.6f}, {longitude:.6f}, {altitude:.1f} m."
            return f"Unknown command: {command_type}"
        except (KeyError, TypeError, ValueError) as exc:
            return f"Invalid command payload: {exc}"
        except Exception as exc:
            logger.exception("Command failed")
            return f"Command failed: {exc}"

    def _send_arm(self, arm: bool) -> None:
        self.vehicle.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1 if arm else 0,
            0,
            0,
            0,
            0,
            0,
            0,
        )

    async def set_mode(self, mode: str) -> str:
        mode = mode.upper()
        allowed_modes = {"GUIDED", "AUTO", "RTL", "LOITER", "STABILIZE"}
        if mode not in allowed_modes:
            return f"Unsupported mode: {mode}"
        if not self.vehicle or not self.connected:
            return "Vehicle is not connected."
        if mode not in self.mode_map:
            return f"Mode is not available for this vehicle: {mode}"
        try:
            mode_id = self.mode_map[mode]
            self.vehicle.mav.set_mode_send(
                self.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
            )
            return f"Mode command sent: {mode}."
        except Exception as exc:
            logger.exception("Mode command failed")
            return f"Mode command failed: {exc}"

    async def ensure_guided(self) -> bool:
        if STATE["mode"] == "GUIDED":
            return True
        result = await self.set_mode("GUIDED")
        if not result.startswith("Mode command sent"):
            return False
        deadline = asyncio.get_running_loop().time() + GUIDED_WAIT_SECONDS
        while asyncio.get_running_loop().time() < deadline:
            if STATE["mode"] == "GUIDED":
                return True
            await asyncio.sleep(0.2)
        return STATE["mode"] == "GUIDED"


app = FastAPI()
app.mount("/static", StaticFiles(directory=FRONTEND_DIR), name="static")

clients: Set[WebSocket] = set()
drone = DroneController()


@app.get("/")
async def index() -> FileResponse:
    return FileResponse(FRONTEND_DIR / "index.html")


@app.get("/register_service")
async def register_service() -> Dict[str, Any]:
    return {
        "name": "Drone Web App",
        "description": "Browser-based MAVLink drone monitor and controller.",
        "icon": "mdi-drone",
        "company": "",
        "version": "1.0.0",
        "webpage": "",
        "api": "/docs",
        "avoid_iframes": True,
    }


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket) -> None:
    await websocket.accept()
    clients.add(websocket)
    await send_json(websocket, {"type": "state", "state": STATE.copy()})
    try:
        while True:
            data = await websocket.receive_text()
            try:
                payload = json.loads(data)
            except json.JSONDecodeError:
                await send_json(websocket, {"type": "status", "message": "Invalid JSON."})
                continue
            message = await drone.command(payload)
            await send_json(websocket, {"type": "status", "message": message})
    except WebSocketDisconnect:
        pass
    finally:
        clients.discard(websocket)


async def send_json(websocket: WebSocket, payload: Dict[str, Any]) -> None:
    await websocket.send_text(json.dumps(payload))


async def broadcast_state() -> None:
    if not clients:
        return
    message = {"type": "state", "state": STATE.copy()}
    disconnected = []
    for websocket in list(clients):
        try:
            await send_json(websocket, message)
        except Exception:
            disconnected.append(websocket)
    for websocket in disconnected:
        clients.discard(websocket)
