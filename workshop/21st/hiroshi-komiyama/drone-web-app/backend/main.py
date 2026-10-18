import asyncio
import json
import math
import os
import queue
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink_common


class DroneController:
    def __init__(self, move_speed: float = 0.5) -> None:
        self.heading = None
        self.move_speed = move_speed

    def build_velocity_vector(self, action: str, heading: float | None = None) -> tuple[float, float, float]:
        heading_value = heading if heading is not None else self.heading
        if heading_value is None:
            raise ValueError("heading not available")

        heading_rad = math.radians(heading_value)
        if action == "moveForward":
            return (round(self.move_speed * math.cos(heading_rad), 10), round(self.move_speed * math.sin(heading_rad), 10), 0.0)
        if action == "moveBack":
            return (round(-self.move_speed * math.cos(heading_rad), 10), round(-self.move_speed * math.sin(heading_rad), 10), 0.0)
        if action == "moveLeft":
            return (round(-self.move_speed * math.sin(heading_rad), 10), round(self.move_speed * math.cos(heading_rad), 10), 0.0)
        if action == "moveRight":
            return (round(self.move_speed * math.sin(heading_rad), 10), round(-self.move_speed * math.cos(heading_rad), 10), 0.0)
        return (0.0, 0.0, 0.0)


BASE_DIR = Path(__file__).resolve().parent.parent
FRONTEND_DIR = Path(__file__).resolve().parent / "frontend"

app = FastAPI(title="Drone Web Controller")
app.mount("/static", StaticFiles(directory=str(FRONTEND_DIR)), name="static")

app.state.loop = None
app.state.executor = None

state_lock = threading.Lock()
state = {
    "connected": False,
    "armed": False,
    "mode": "UNKNOWN",
    "latitude": 0.0,
    "longitude": 0.0,
    "altitude": 0.0,
    "heading": 0,
}

clients = set()
command_queue = queue.Queue()
current_vehicle = None
current_target_system = None
current_target_component = None
MODE_MAP = {}  # Explicit mode map (set at connection time)
mav_thread = None
mav_worker_running = False
active_move_command = None
last_move_send = 0.0
controller = DroneController()


def get_state_snapshot():
    with state_lock:
        return dict(state)


def update_state(new_state):
    global state
    with state_lock:
        state.update(new_state)
    broadcast_state()


def set_status(message):
    async def _send():
        if app.state.loop is None:
            return
        await _broadcast_json({"type": "status", "message": message})

    if app.state.loop is not None:
        app.state.loop.call_soon_threadsafe(lambda: asyncio.create_task(_send()))


async def _broadcast_json(payload):
    dead_clients = []
    for ws in list(clients):
        try:
            await ws.send_json(payload)
        except Exception:
            dead_clients.append(ws)
    for ws in dead_clients:
        clients.discard(ws)


def broadcast_state():
    payload = {"type": "state", "state": get_state_snapshot()}

    if app.state.loop is None:
        return
    app.state.loop.call_soon_threadsafe(lambda: asyncio.create_task(_broadcast_json(payload)))


@app.on_event("startup")
def startup_event():
    app.state.loop = asyncio.get_running_loop()
    app.state.executor = ThreadPoolExecutor(max_workers=2)


@app.on_event("shutdown")
def shutdown_event():
    global mav_worker_running, current_vehicle
    mav_worker_running = False
    if current_vehicle is not None:
        try:
            current_vehicle.close()
        except Exception:
            pass
    if app.state.executor is not None:
        app.state.executor.shutdown(wait=False, cancel_futures=True)


@app.get("/")
async def index():
    return FileResponse(FRONTEND_DIR / "index.html")


@app.get("/register_service")
async def register_service():
    """BlueOS service registration endpoint."""
    return {
        "name": "Drone Web App",
        "description": "FastAPI-based drone web application for MAVLink control",
        "icon": "mdi-drone",
        "company": "",
        "version": "1.0.0",
        "webpage": "",
        "api": "/docs",
        "avoid_iframes": True,  # WebSocket: open in new window (direct http://<IP>:<PORT>/)
    }


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    clients.add(websocket)
    await websocket.send_json({"type": "state", "state": get_state_snapshot()})
    await websocket.send_json({"type": "status", "message": "WebSocket connected"})

    try:
        while True:
            try:
                raw_message = await websocket.receive_text()
            except WebSocketDisconnect:
                break
            except Exception as exc:
                print(f"Receive error: {exc}")
                continue

            if not raw_message:
                continue

            try:
                message = json.loads(raw_message)
            except json.JSONDecodeError:
                await websocket.send_json({"type": "status", "message": "invalid command"})
                continue

            command_type = message.get("type")
            if not command_type:
                continue

            if command_type == "connect":
                start_mavlink_worker()
                await websocket.send_json({"type": "status", "message": "Connection requested"})
            else:
                if should_start_worker(command_type, mav_worker_running):
                    start_mavlink_worker()
                command_queue.put(message)
                await websocket.send_json({"type": "status", "message": f"Command queued: {command_type}"})
    except WebSocketDisconnect:
        clients.discard(websocket)
    except Exception as exc:
        clients.discard(websocket)
        print(f"WebSocket error: {exc}")


def should_start_worker(command_type: str, worker_running: bool) -> bool:
    return command_type != "connect" and not worker_running


def start_mavlink_worker():
    global mav_thread, mav_worker_running
    if mav_thread is not None and not mav_thread.done():
        return
    mav_worker_running = True
    if app.state.executor is None:
        app.state.executor = ThreadPoolExecutor(max_workers=2)
    mav_thread = app.state.executor.submit(mavlink_worker)


#def get_connection_string() -> str:
#    return os.getenv("MAV_ENDPOINT") or "udpout:host.docker.internal:14550"
#def get_connection_string() -> str:
#    return "udp:host.docker.internal:14550"

#def get_connection_string() -> str:
#    return os.environ.get(
#        "MAV_ENDPOINT",
#        "udpout:host.docker.internal:14550"
#    )

def get_connection_string() -> str:
    return os.environ.get(
        "MAV_ENDPOINT",
        "udpout:host.docker.internal:14550"
    )

def resolve_command_target(vehicle):
    target_system = current_target_system
    target_component = current_target_component
    if target_system is None:
        target_system = getattr(vehicle, "target_system", 1)
    if target_component is None:
        target_component = getattr(vehicle, "target_component", 1)
    if target_system in (None, 0):
        target_system = 1
    if target_component in (None, 0):
        target_component = 1
    return int(target_system), int(target_component)


def update_target_from_message(vehicle, msg) -> None:
    global current_target_system, current_target_component
    src_system = getattr(msg, "get_srcSystem", lambda: None)()
    src_component = getattr(msg, "get_srcComponent", lambda: None)()
    if src_system is None or src_component is None:
        return
    current_target_system = int(src_system)
    current_target_component = int(src_component)
    setattr(vehicle, "target_system", int(src_system))
    setattr(vehicle, "target_component", int(src_component))


def send_takeoff_command(vehicle, target_system, target_component, altitude: float) -> None:
    vehicle.mav.command_long_send(
        target_system,
        target_component,
        mavlink_common.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        float(altitude),
    )


def get_mode_name(vehicle):
    global MODE_MAP
    if vehicle is None or not MODE_MAP:
        return "UNKNOWN"
    try:
        custom_mode = getattr(vehicle, "mode", None)
        if custom_mode is None:
            return "UNKNOWN"
        for name, value in MODE_MAP.items():
            if value == custom_mode:
                return name.upper()
    except Exception:
        pass
    return "UNKNOWN"


def ensure_guided(vehicle):
    global MODE_MAP
    if vehicle is None or not MODE_MAP:
        return False
    try:
        if getattr(vehicle, "mode", None) is not None:
            current_mode = get_mode_name(vehicle)
            if current_mode == "GUIDED":
                return True
    except Exception:
        pass

    try:
        if "GUIDED" not in MODE_MAP:
            print(f"GUIDED mode not in MODE_MAP: {list(MODE_MAP.keys())}")
            return False
        mode_id = MODE_MAP["GUIDED"]
        vehicle.set_mode(mode_id)
        print(f"GUIDED transition: set_mode({mode_id})")
    except Exception as exc:
        print(f"GUIDED transition failed: {exc}")
    return True


def send_velocity_command(vehicle, cmd):
    global current_target_system, current_target_component, active_move_command
    if vehicle is None or current_target_system is None or current_target_component is None:
        return

    if not ensure_guided(vehicle):
        set_status("GUIDED mode required for relative motion")
        return

    if cmd == "moveForward":
        vx, vy, vz = 0.5, 0.0, 0.0
    elif cmd == "moveBack":
        vx, vy, vz = -0.5, 0.0, 0.0
    elif cmd == "moveLeft":
        vx, vy, vz = 0.0, 0.5, 0.0
    elif cmd == "moveRight":
        vx, vy, vz = 0.0, -0.5, 0.0
    else:
        vx, vy, vz = 0.0, 0.0, 0.0

    heading = get_state_snapshot().get("heading", 0) or 0
    controller.heading = float(heading)
    vx, vy, vz = controller.build_velocity_vector(cmd, float(heading))

    try:
        print(f"Sending velocity command: {cmd} vx={vx} vy={vy} vz={vz} target=({current_target_system},{current_target_component})")
        vehicle.mav.set_position_target_local_ned_send(
            0,
            current_target_system,
            current_target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0,
            0.0,
            0.0,
            0.0,
            vx,
            vy,
            vz,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
    except Exception as exc:
        print(f"Velocity send failed: {exc}")


def send_stop_command(vehicle):
    global current_target_system, current_target_component, active_move_command
    if vehicle is None or current_target_system is None or current_target_component is None:
        return
    active_move_command = None
    try:
        print("Sending stop command")
        vehicle.mav.set_position_target_local_ned_send(
            0,
            current_target_system,
            current_target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
    except Exception as exc:
        print(f"Stop command failed: {exc}")


def mavlink_worker():
    global current_vehicle, current_target_system, current_target_component, mav_worker_running, active_move_command, last_move_send, mav_thread, MODE_MAP

    connection_string = get_connection_string()
    print(f"[MAVLINK] Connecting to {connection_string}", file=sys.stderr, flush=True)

    try:
        vehicle = mavutil.mavlink_connection(connection_string)
        print(f"[MAVLINK] Connection object created, waiting for heartbeat...", file=sys.stderr, flush=True)

#----
        print(
            "[MAVLINK] Waiting for ArduPilot heartbeat...",
            file=sys.stderr,
            flush=True,
        )

        hb = None
        deadline = time.time() + 15

        while time.time() < deadline:
            msg = vehicle.recv_match(
                type="HEARTBEAT",
                blocking=True,
                timeout=1
            )

            if not msg:
                continue
            
            print(
                f"[HB] "
                f"sysid={msg.get_srcSystem()} "
                f"compid={msg.get_srcComponent()} "
                f"type={msg.type} "
                f"autopilot={msg.autopilot}",
                file=sys.stderr,
                flush=True,
            )

            if msg.autopilot == mavlink_common.MAV_AUTOPILOT_ARDUPILOTMEGA:
                hb = msg
                break
            
        if hb is None:
            print(
                "[MAVLINK] No ArduPilot heartbeat",
                file=sys.stderr,
                flush=True
            )
            return
        #----

        if hb is None:
            print("[MAVLINK] No autopilot heartbeat received", file=sys.stderr, flush=True)
            update_state({"connected": False})
            set_status("No autopilot heartbeat")
            mav_worker_running = False
            return
        
        # Generate explicit mode map from autopilot type
        MODE_MAP = mavutil.mode_mapping_byname(hb.type) or {}
        print(f"[MAVLINK] Autopilot type={hb.type}, modes={list(MODE_MAP.keys())}", file=sys.stderr, flush=True)
        
    except Exception as exc:
        print(f"[MAVLINK] Connect failed: {exc}", file=sys.stderr, flush=True)
        update_state({"connected": False})
        set_status("MAVLink connection failed")
        mav_worker_running = False
        return

    current_vehicle = vehicle
    current_target_system = getattr(vehicle, "target_system", 1) or 1
    current_target_component = getattr(vehicle, "target_component", 1) or 1
    update_state({"connected": True})
    set_status("MAVLink connected")

    try:
        vehicle.mav.request_data_stream_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            10,
            1,
        )
        vehicle.mav.request_data_stream_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
            10,
            1,
        )
    except Exception as exc:
        print(f"Stream request failed: {exc}")

    while mav_worker_running:
        try:
            msg = vehicle.recv_match(blocking=True, timeout=0.1)
            if msg is None:
                pass
            else:
                msg_type = msg.get_type()
                if msg_type == "GLOBAL_POSITION_INT":
                    lat = msg.lat / 1e7
                    lon = msg.lon / 1e7
                    alt = (msg.relative_alt / 1000.0) if getattr(msg, "relative_alt", None) is not None else (msg.alt / 1000.0 if getattr(msg, "alt", None) is not None else 0.0)
                    heading = msg.hdg / 100.0 if getattr(msg, "hdg", None) not in (None, 65535) else None
                    update_state({
                        "latitude": lat,
                        "longitude": lon,
                        "altitude": alt,
                        "heading": int(heading) if heading is not None else get_state_snapshot().get("heading", 0),
                    })
                elif msg_type == "HEARTBEAT":
                    if getattr(msg, "type", None) == mavlink_common.MAV_TYPE_GCS or getattr(msg, "autopilot", None) == mavlink_common.MAV_AUTOPILOT_INVALID:
                        continue
                    update_target_from_message(vehicle, msg)
                    if getattr(msg, "system_status", None) is not None:
                        armed = bool(msg.base_mode & mavlink_common.MAV_MODE_FLAG_SAFETY_ARMED)
                        update_state({"armed": armed})
                        
                        # Extract mode from HEARTBEAT custom_mode
                        mode_name = "UNKNOWN"
                        try:
                            custom_mode = getattr(msg, "custom_mode", None)
                            if custom_mode is not None and MODE_MAP:
                                for name, value in MODE_MAP.items():
                                    if value == custom_mode:
                                        mode_name = name.upper()
                                        break
                                print(f"[MODE] custom_mode={custom_mode}, name={mode_name}", file=sys.stderr, flush=True)
                        except Exception as e:
                            print(f"[MODE_ERROR] {e}", file=sys.stderr, flush=True)
                        update_state({"mode": mode_name})

            while not command_queue.empty():
                try:
                    command = command_queue.get_nowait()
                except queue.Empty:
                    break

                command_type = command.get("type")

                if command_type == "arm":
                    try:
                        target_system, target_component = resolve_command_target(vehicle)
                        vehicle.mav.command_long_send(
                            target_system,
                            target_component,
                            mavlink_common.MAV_CMD_COMPONENT_ARM_DISARM,
                            0,
                            1,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                        )
                        set_status("Armed")
                    except Exception as exc:
                        set_status(f"Arm failed: {exc}")

                elif command_type == "disarm":
                    try:
                        target_system, target_component = resolve_command_target(vehicle)
                        vehicle.mav.command_long_send(
                            target_system,
                            target_component,
                            mavlink_common.MAV_CMD_COMPONENT_ARM_DISARM,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                        )
                        set_status("Disarmed")
                    except Exception as exc:
                        set_status(f"Disarm failed: {exc}")

                elif command_type == "takeoff":
                    try:
                        altitude = float(command.get("altitude", 0.0))
                        target_system, target_component = resolve_command_target(vehicle)
                        if ensure_guided(vehicle):
                            send_takeoff_command(vehicle, target_system, target_component, altitude)
                            set_status(f"Takeoff requested at {altitude} m")
                        else:
                            send_takeoff_command(vehicle, target_system, target_component, altitude)
                            set_status(f"Takeoff requested at {altitude} m (GUIDED transition fallback)")
                    except Exception as exc:
                        set_status(f"Takeoff failed: {exc}")

                elif command_type == "land":
                    try:
                        active_move_command = None
                        target_system, target_component = resolve_command_target(vehicle)
                        vehicle.mav.command_long_send(
                            target_system,
                            target_component,
                            mavlink_common.MAV_CMD_NAV_LAND,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                        )
                        set_status("Land requested")
                    except Exception as exc:
                        set_status(f"Land failed: {exc}")

                elif command_type == "goto":
                    try:
                        target_system, target_component = resolve_command_target(vehicle)
                        if ensure_guided(vehicle):
                            lat = int(float(command.get("latitude", 0.0)) * 1e7)
                            lon = int(float(command.get("longitude", 0.0)) * 1e7)
                            alt = int(float(command.get("altitude", 0.0)) * 1000)
                            vehicle.mav.set_position_target_global_int_send(
                                0,
                                target_system,
                                target_component,
                                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                0b0000111111111000,
                                lat,
                                lon,
                                alt,
                                0,
                                0,
                                0,
                                0,
                                0,
                                0,
                                0,
                                0,
                            )
                            set_status("GoTo requested")
                        else:
                            set_status("GUIDED transition failed")
                    except Exception as exc:
                        set_status(f"Goto failed: {exc}")

                elif command_type == "mode":
                    try:
                        active_move_command = None
                        target_mode = command.get("mode", "GUIDED").upper()
                        if target_mode not in MODE_MAP:
                            set_status(f"Mode {target_mode} not available")
                            print(f"[MODE_CMD] Requested mode not in MAP: {target_mode}, available: {list(MODE_MAP.keys())}", file=sys.stderr, flush=True)
                        else:
                            mode_id = MODE_MAP[target_mode]
                            vehicle.set_mode(mode_id)
                            update_state({"mode": target_mode})
                            set_status(f"Mode set to {target_mode}")
                            print(f"[MODE_CMD] Mode changed to {target_mode} (id={mode_id})", file=sys.stderr, flush=True)
                    except Exception as exc:
                        set_status(f"Mode change failed: {exc}")
                        print(f"[MODE_CMD_ERROR] {exc}", file=sys.stderr, flush=True)

                elif command_type in {"moveForward", "moveBack", "moveLeft", "moveRight"}:
                    active_move_command = command_type
                    set_status(f"Moving {command_type}")
                    send_velocity_command(vehicle, command_type)

                elif command_type == "moveStop":
                    active_move_command = None
                    send_stop_command(vehicle)
                    set_status("Movement stopped")

                time.sleep(0.01)

            if active_move_command is not None:
                if time.monotonic() - last_move_send >= 0.1:
                    send_velocity_command(vehicle, active_move_command)
                    last_move_send = time.monotonic()

            time.sleep(0.01)
        except Exception as exc:
            print(f"MAVLink loop error: {exc}")
            set_status("MAVLink loop error")
            time.sleep(0.2)

    try:
        vehicle.close()
    except Exception:
        pass
    current_vehicle = None
    current_target_system = None
    current_target_component = None
    update_state({"connected": False})
    set_status("MAVLink disconnected")
    mav_thread = None
