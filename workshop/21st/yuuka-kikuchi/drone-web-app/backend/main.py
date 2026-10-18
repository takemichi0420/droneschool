import os
import asyncio
import json
import time
import functools
import threading
import logging
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from pymavlink import mavutil

# Configure logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

app = FastAPI()

# Mount static files for the frontend
app.mount("/static", StaticFiles(directory="../frontend"), name="static")

# Drone connection global variables
connection_string = os.environ.get("MAV_ENDPOINT", "tcp:127.0.0.1:5762")
vehicle = None
drone_connected = False
MODE_MAP = {}
REVERSE_MODE_MAP = {}
drone_status = {
    "connected": False,
    "armed": False,
    "mode": "UNKNOWN",
    "latitude": 0.0,
    "longitude": 0.0,
    "altitude": 0.0,
    "heading": 0,
}
vehicle_lock = threading.Lock()  # Protect vehicle state access
websocket_clients = []  # Track connected WebSocket clients

# --- MAVLink Helper Functions (adapted from CLI app) ---
async def request_data_streams():
    if not vehicle or not drone_connected:
        return

    logger.info("Requesting data streams...")
    try:
        # Request position data stream at 10 Hz
        vehicle.mav.request_data_stream_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            10,  # Rate in Hz
            1)   # Start sending
    except Exception as e:
        logger.error(f"Error requesting data streams: {e}")

async def broadcast_status():
    """Broadcast current drone status to all connected WebSocket clients."""
    global websocket_clients
    disconnected = []
    for i, client in enumerate(websocket_clients):
        try:
            await client.send_json({"type": "state", "state": drone_status})
        except Exception as e:
            logger.debug(f"Error sending to client {i}: {e}")
            disconnected.append(i)
    
    # Remove disconnected clients
    for i in reversed(disconnected):
        websocket_clients.pop(i)

def connect_to_vehicle():
    """Start a background thread that attempts to connect to the vehicle without blocking the main thread."""
    global vehicle, drone_connected, MODE_MAP, REVERSE_MODE_MAP
    
    def _connect():
        global vehicle, MODE_MAP, REVERSE_MODE_MAP, drone_connected
        retry_count = 0
        
        # Infinite retry loop: keep attempting connection until successful
        while True:
            try:
                retry_count += 1
                logger.info(f"[Attempt {retry_count}] Attempting to connect to vehicle on: {connection_string}")
                m = mavutil.mavlink_connection(connection_string, timeout=5)
                logger.info("MAVLink connection object created, waiting for heartbeat...")
                
                # Send a GCS heartbeat to prompt autopilots to respond
                try:
                    for _ in range(3):
                        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                           mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                        time.sleep(0.1)
                except Exception as e:
                    logger.debug(f"Error sending GCS heartbeat: {e}")
                
                # Wait for HEARTBEAT from autopilot
                hb = None
                deadline = time.time() + 15  # 15 second timeout
                heartbeat_count = 0
                
                while time.time() < deadline:
                    try:
                        msg = m.recv_match(type="HEARTBEAT", blocking=False, timeout=0.5)
                        if msg:
                            logger.debug(f"Received HEARTBEAT: type={msg.type}, autopilot={msg.autopilot}, "
                                       f"system={msg.get_srcSystem()}, component={msg.get_srcComponent()}")
                            
                            # Filter for valid autopilot (not GCS)
                            if (hasattr(msg, 'autopilot') and msg.autopilot is not None and 
                                msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID):
                                hb = msg
                                heartbeat_count += 1
                                # Accept after receiving 2 valid heartbeats for confirmation
                                if heartbeat_count >= 2:
                                    break
                        else:
                            time.sleep(0.01)
                    except Exception as e:
                        logger.debug(f"Error receiving HEARTBEAT: {e}")
                        time.sleep(0.1)
                
                if hb is None:
                    logger.warning(f"No valid HEARTBEAT found after 15s, retrying in 5s...")
                    try:
                        m.close()
                    except Exception:
                        pass
                    time.sleep(5)
                    continue
                
                # Successfully got heartbeat
                with vehicle_lock:
                    m.target_system = hb.get_srcSystem()
                    m.target_component = hb.get_srcComponent()
                    
                    try:
                        MODE_MAP = mavutil.mode_mapping_byname(hb.type) or {}
                        REVERSE_MODE_MAP = {v: k for k, v in MODE_MAP.items()}
                        logger.info(f"Modes available: {list(MODE_MAP.keys())}")
                    except Exception as e:
                        logger.error(f"Error mapping modes: {e}")
                        MODE_MAP = {}
                        REVERSE_MODE_MAP = {}
                    
                    # Request data streams
                    try:
                        m.mav.request_data_stream_send(
                            m.target_system,
                            m.target_component,
                            mavutil.mavlink.MAV_DATA_STREAM_ALL,
                            4,  # 4Hz
                            1)  # Start
                        logger.info("Data streams requested")
                    except Exception as e:
                        logger.error(f"Error requesting data streams: {e}")
                    
                    vehicle = m
                    drone_connected = True
                    drone_status["connected"] = True
                    logger.info(f"✓ Connected to vehicle (system {m.target_system}, component {m.target_component})")
                
                # Broadcast connection status to all clients
                asyncio.run_coroutine_threadsafe(broadcast_status(), asyncio.get_event_loop())
                return True
                
            except ConnectionRefusedError as e:
                logger.warning(f"Connection refused: {e}. Retrying in 5s...")
                time.sleep(5)
            except TimeoutError as e:
                logger.warning(f"Connection timeout: {e}. Retrying in 5s...")
                time.sleep(5)
            except Exception as e:
                logger.error(f"Connection error: {type(e).__name__}: {e}. Retrying in 5s...")
                time.sleep(5)
    
    t = threading.Thread(target=_connect, daemon=True)
    t.start()
    return True

async def wait_for_altitude(target_altitude, tolerance=0.5, timeout=30):
    """Wait for drone to reach target altitude with tolerance."""
    global drone_status
    start_time = time.time()
    while time.time() - start_time < timeout:
        current_alt = drone_status.get("altitude", 0)
        if abs(current_alt - target_altitude) <= tolerance:
            logger.info(f"✓ Altitude reached: {current_alt:.2f}m (target: {target_altitude}m)")
            return True
        await asyncio.sleep(0.5)
    logger.warning(f"Altitude change timeout: current {drone_status.get('altitude', 0):.2f}m, target {target_altitude}m")
    return False

async def wait_for_armed_status(target_armed, timeout=5):
    """Wait for drone to reach target armed status."""
    global drone_status
    start_time = time.time()
    while time.time() - start_time < timeout:
        if drone_status.get("armed") == target_armed:
            status_str = "armed" if target_armed else "disarmed"
            logger.info(f"✓ Drone {status_str}")
            return True
        await asyncio.sleep(0.2)
    return False

async def set_mode(mode_name):
    if not vehicle or not drone_connected:
        logger.warning(f"Cannot set mode {mode_name}: vehicle not connected")
        return False

    logger.info(f"Setting mode to {mode_name}...")
    
    # Use cached MODE_MAP from connection time
    if not MODE_MAP or mode_name not in MODE_MAP:
        logger.error(f"Unknown mode: {mode_name}")
        logger.info("Available modes: %s" % list(MODE_MAP.keys()) if MODE_MAP else "No modes loaded")
        return False

    try:
        mode_id = MODE_MAP[mode_name]
        logger.debug(f"Mode '{mode_name}' -> ID {mode_id}")
        
        # Send set_mode command via MAVLink command_long
        vehicle.mav.command_long_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,  # confirmation
            1,  # mode_param1 (base mode - 1 = use custom_mode)
            mode_id,  # mode_param2 (custom mode)
            0, 0, 0, 0, 0
        )
        logger.info(f"Mode change command sent for {mode_name} (ID {mode_id}).")
        
        # Wait for mode to be confirmed (up to 5 seconds)
        return await wait_for_mode(mode_name, timeout=5)
    except Exception as e:
        logger.error(f"Error setting mode: {e}")
        return False

async def wait_for_mode(target_mode, timeout=5):
    """Wait for drone to reach target mode with timeout (in seconds)."""
    global drone_status
    start_time = time.time()
    while time.time() - start_time < timeout:
        if drone_status.get("mode") == target_mode:
            logger.info(f"✓ Mode reached: {target_mode}")
            return True
        await asyncio.sleep(0.2)
    logger.warning(f"Mode change timeout: expected {target_mode}, got {drone_status.get('mode')}")
    return False

async def arm_vehicle():
    """Arm the vehicle without changing its current mode."""
    if not vehicle or not drone_connected:
        logger.warning("Cannot arm: vehicle not connected")
        return

    try:
        logger.info("Arming motors (without mode change)...")
        vehicle.mav.command_long_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0)
        logger.info("Arm command sent.")
        
        # Wait for arm confirmation
        await wait_for_armed_status(True, timeout=5)
    except Exception as e:
        logger.error(f"Error arming: {e}")

async def takeoff_vehicle(altitude):
    global pre_arm_mode
    if not vehicle or not drone_connected:
        logger.warning("Cannot takeoff: vehicle not connected")
        return

    try:
        # TAKEOFF command only works in GUIDED (or AUTO) mode
        # If currently in another mode, switch to GUIDED first
        current_mode = drone_status.get("mode", "UNKNOWN")
        if current_mode not in ["GUIDED", "AUTO"]:
            logger.info(f"Current mode {current_mode} doesn't support TAKEOFF. Switching to GUIDED...")
            if not await set_mode("GUIDED"):
                logger.error("Failed to set GUIDED mode for takeoff. Aborting takeoff.")
                return

        logger.info(f"Taking off to altitude: {altitude} meters")
        vehicle.mav.command_long_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0, altitude)
        logger.info("Takeoff command sent, waiting for altitude...")
        
        # Wait for drone to reach target altitude
        await wait_for_altitude(altitude, tolerance=1.0, timeout=60)
        logger.info(f"✓ Takeoff to {altitude}m completed")
    except Exception as e:
        logger.error(f"Error taking off: {e}")

async def land_vehicle():
    if not vehicle or not drone_connected:
        logger.warning("Cannot land: vehicle not connected")
        return

    try:
        logger.info("Landing vehicle...")
        
        # LAND command works in GUIDED or AUTO mode
        # If currently in another mode (e.g., LOITER), switch to GUIDED first
        current_mode = drone_status.get("mode", "UNKNOWN")
        if current_mode not in ["GUIDED", "AUTO"]:
            logger.info(f"Current mode {current_mode} doesn't support LAND. Switching to GUIDED...")
            if not await set_mode("GUIDED"):
                logger.error("Failed to set GUIDED mode for landing. Aborting land.")
                return
        
        # Send MAV_CMD_NAV_LAND command
        vehicle.mav.command_long_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,  # confirmation
            0,  # param1: abort altitude (0 = use default)
            0,  # param2: precision land mode
            0,  # param3: empty
            0,  # param4: yaw angle
            0,  # param5: lat
            0,  # param6: lon
            0)  # param7: alt
        logger.info("Land command sent, waiting for drone to land...")
        
        # Wait for drone to land (altitude near 0)
        landed = await wait_for_altitude(0.5, tolerance=0.5, timeout=60)
        
        if landed:
            logger.info("✓ Drone landed")
            # Auto-disarm after landing
            await asyncio.sleep(1)  # Wait 1 second before disarming
            logger.info("Auto-disarming...")
            await disarm_vehicle()
        else:
            logger.warning("Landing timeout - drone may not have landed completely")
    except Exception as e:
        logger.error(f"Error landing: {e}")

async def disarm_vehicle():
    """Disarm the vehicle."""
    if not vehicle or not drone_connected:
        logger.warning("Cannot disarm: vehicle not connected")
        return

    try:
        logger.info("Disarming motors...")
        vehicle.mav.command_long_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0)
        logger.info("Disarm command sent.")
        
        # Wait for drone to be disarmed
        await wait_for_armed_status(False, timeout=5)
    except Exception as e:
        logger.error(f"Error disarming: {e}")

async def goto_location(latitude, longitude, altitude):
    if not vehicle or not drone_connected:
        logger.warning("Cannot goto: vehicle not connected")
        return

    # For goto, we need GUIDED mode
    current_mode = drone_status.get("mode", "UNKNOWN")
    if current_mode != "GUIDED":
        logger.info(f"Switching to GUIDED for goto (was {current_mode})")
        if not await set_mode("GUIDED"):
            logger.error("Failed to set GUIDED mode. Cannot go to location.")
            return

    try:
        logger.info(f"Moving to Lat: {latitude}, Lon: {longitude}, Alt: {altitude}")
        vehicle.mav.set_position_target_global_int_send(
            0,       # time_boot_ms (not used)
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000, # type_mask (only position enabled)
            int(latitude * 1e7),
            int(longitude * 1e7),
            altitude,
            0,       # vx
            0,       # vy
            0,       # vz
            0, 0, 0, # afx, afy, afz (not used)
            0, 0)    # yaw, yaw_rate (not used)
        logger.info("Go-to command sent.")
    except Exception as e:
        logger.error(f"Error going to location: {e}")

# --- WebSocket Endpoint ---
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    logger.info("WebSocket connected.")
    websocket_clients.append(websocket)
    try:
        # Send initial drone status
        await websocket.send_json({"type": "state", "state": drone_status})
        logger.debug(f"Initial status sent. Total clients: {len(websocket_clients)}")

        # Task to continuously read MAVLink messages and send status
        async def mavlink_reader():
            global drone_status
            loop = asyncio.get_event_loop()
            last_heartbeat = time.time()
            
            while True:
                try:
                    if vehicle and drone_connected:
                        # Use run_in_executor to avoid blocking the event loop.
                        msg = await loop.run_in_executor(
                            None, functools.partial(vehicle.recv_match, blocking=False, timeout=0.1)
                        )
                        if msg:
                            # Only process messages from the connected vehicle
                            try:
                                src_sys = msg.get_srcSystem()
                                src_comp = msg.get_srcComponent()
                            except Exception:
                                src_sys = None
                                src_comp = None
                            
                            if vehicle and src_sys == getattr(vehicle, "target_system", None) and src_comp == getattr(vehicle, "target_component", None):
                                # Update drone_status based on MAVLink messages
                                if msg.get_type() == 'GLOBAL_POSITION_INT':
                                    drone_status["latitude"] = msg.lat / 1e7
                                    drone_status["longitude"] = msg.lon / 1e7
                                    drone_status["altitude"] = msg.relative_alt / 1000.0
                                    if msg.hdg != 65535:
                                        drone_status["heading"] = msg.hdg / 100.0
                                    await broadcast_status()
                                    
                                elif msg.get_type() == 'HEARTBEAT':
                                    # ARM status via base_mode SAFETY_ARMED flag
                                    try:
                                        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                                    except Exception:
                                        armed = drone_status.get("armed", False)
                                    
                                    old_armed = drone_status.get("armed", False)
                                    old_mode = drone_status.get("mode", "UNKNOWN")
                                    
                                    drone_status["armed"] = armed
                                    mode_name = REVERSE_MODE_MAP.get(msg.custom_mode, "UNKNOWN")
                                    drone_status["mode"] = mode_name
                                    
                                    # Only broadcast if something changed
                                    if old_armed != armed or old_mode != mode_name:
                                        logger.debug(f"State change: armed={armed}, mode={mode_name}")
                                        await broadcast_status()
                                    
                                    last_heartbeat = time.time()
                        else:
                            await asyncio.sleep(0.01)
                    else:
                        # If not connected, wait a bit before checking again
                        await asyncio.sleep(1)
                except WebSocketDisconnect:
                    logger.info("MAVLink reader: WebSocket disconnected")
                    break
                except Exception as e:
                    logger.error(f"Error in mavlink_reader: {e}")
                    await asyncio.sleep(0.1)

        reader_task = asyncio.create_task(mavlink_reader())

        while True:
            try:
                data = await websocket.receive_text()
                command = json.loads(data)
                logger.info(f"Received command: {command}")

                # Handle commands from frontend
                if command["type"] == "connect":
                    if not drone_connected:
                        logger.info("Connection button pressed, initiating MAVLink connection...")
                        connect_to_vehicle()
                    await websocket.send_json({"type": "status", "message": "Connection attempt initiated."})
                    
                elif command["type"] == "arm":
                    await arm_vehicle()
                    await websocket.send_json({"type": "status", "message": "Arm command sent."})
                    
                elif command["type"] == "disarm":
                    logger.info("Disarm command requested")
                    await disarm_vehicle()
                    await websocket.send_json({"type": "status", "message": "✓ Disarm command sent."})
                            
                elif command["type"] == "takeoff":
                    altitude = float(command["altitude"])
                    logger.info(f"Takeoff command requested to {altitude}m")
                    await takeoff_vehicle(altitude)
                    await websocket.send_json({"type": "status", "message": f"✓ Takeoff to {altitude}m completed."})
                    
                elif command["type"] == "land":
                    logger.info("Land command requested")
                    await land_vehicle()
                    await websocket.send_json({"type": "status", "message": "✓ Land and auto-disarm completed."})
                    
                elif command["type"] == "goto":
                    lat = float(command["latitude"])
                    lon = float(command["longitude"])
                    alt = float(command["altitude"])
                    await goto_location(lat, lon, alt)
                    await websocket.send_json({"type": "status", "message": f"GoTo {lat},{lon},{alt} command sent."})
                    
                elif command["type"] == "mode":
                    mode_name = command["mode_name"].upper()
                    logger.info(f"Mode change requested: {mode_name}")
                    mode_changed = await set_mode(mode_name)
                    if mode_changed:
                        await websocket.send_json({"type": "status", "message": f"✓ Mode changed to {mode_name}"})
                    else:
                        await websocket.send_json({"type": "status", "message": f"✗ Failed to change mode to {mode_name}"})
                else:
                    logger.warning(f"Unknown command type: {command.get('type')}")
                    await websocket.send_json({"type": "status", "message": "Unknown command"})

            except json.JSONDecodeError as e:
                logger.error(f"JSON decode error: {e}")
                await websocket.send_json({"type": "status", "message": "Invalid JSON"})
            except Exception as e:
                logger.error(f"Error processing command: {e}")
                await websocket.send_json({"type": "status", "message": f"Error: {e}"})

    except WebSocketDisconnect:
        logger.info("WebSocket disconnected.")
        if websocket in websocket_clients:
            websocket_clients.remove(websocket)
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
        if websocket in websocket_clients:
            websocket_clients.remove(websocket)
    finally:
        if 'reader_task' in locals() and not reader_task.done():
            reader_task.cancel()
        logger.info(f"WebSocket cleanup done. Remaining clients: {len(websocket_clients)}")

# --- HTTP Endpoint for Frontend ---
@app.get("/")
async def get_frontend():
    # Serve the index.html file from the frontend directory
    try:
        with open("../frontend/index.html", "r", encoding="utf-8") as f:
            return HTMLResponse(content=f.read())
    except FileNotFoundError:
        logger.error("index.html not found in ../frontend/")
        return HTMLResponse(content="<h1>Error: Frontend not found</h1>", status_code=404)

# Register service for BlueOS
@app.get("/register_service")
async def register_service():
    return {
        "name": "Drone Web App",
        "description": "Drone Web App — FastAPI + pymavlink web UI",
        "icon": "mdi-drone",
        "company": "",
        "version": "1.0.0",
        "webpage": "",
        "api": "/docs",
        "avoid_iframes": True,
    }

# --- Startup Event ---
@app.on_event("startup")
async def startup_event():
    # Attempt to connect to the drone on startup
    # For a real application, this might be triggered by a user action
    # connect_to_vehicle() # Don't auto-connect, let frontend trigger it
    pass