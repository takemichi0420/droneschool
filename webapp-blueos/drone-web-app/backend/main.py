import asyncio
import json
import time
import functools
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from pymavlink import mavutil

app = FastAPI()

# Mount static files for the frontend
app.mount("/static", StaticFiles(directory="../frontend"), name="static")

# Drone connection global variables
connection_string = 'tcp:127.0.0.1:5762'
vehicle = None
drone_connected = False
drone_status = {
    "connected": False,
    "armed": False,
    "mode": "UNKNOWN",
    "latitude": 0.0,
    "longitude": 0.0,
    "altitude": 0.0,
    "heading": 0,
}

# --- MAVLink Helper Functions (adapted from CLI app) ---
async def request_data_streams():
    if not vehicle or not drone_connected:
        return

    print("Requesting data streams...")
    # Request position data stream at 10 Hz
    vehicle.mav.request_data_stream_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,
        10,  # Rate in Hz
        1)   # Start sending

def connect_to_vehicle():
    global vehicle, drone_connected
    print(f"Attempting to connect to vehicle on: {connection_string}")
    try:
        vehicle = mavutil.mavlink_connection(connection_string, wait_heartbeat=True)
        vehicle.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % (vehicle.target_system, vehicle.target_component))
        drone_connected = True
        drone_status["connected"] = True
        # Request data streams after successful connection
        asyncio.create_task(request_data_streams()) # Schedule as a task
        return True
    except Exception as e:
        print(f"Failed to connect to vehicle: {e}")
        drone_connected = False
        drone_status["connected"] = False
        return False

async def set_mode(mode_name):
    if not vehicle or not drone_connected:
        return False

    print(f"Setting mode to {mode_name}...")
    if mode_name not in vehicle.mode_mapping():
        print(f"Unknown mode: {mode_name}")
        print("Available modes: ", list(vehicle.mode_mapping().keys()))
        return False

    mode_id = vehicle.mode_mapping()[mode_name]
    vehicle.set_mode(mode_id)
    # Don't sleep here. Let the mavlink_reader report the mode change.
    print(f"Mode change command sent for {mode_name}.")
    return True

async def arm_vehicle():
    if not vehicle or not drone_connected:
        return

    if not await set_mode("GUIDED"):
        print("Failed to set GUIDED mode. Cannot arm.")
        return

    print("Arming motors...")
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)
    # Don't sleep or assume success. The mavlink_reader will update the armed status
    # based on HEARTBEAT messages from the drone.
    print("Arm command sent.")

async def takeoff_vehicle(altitude):
    if not vehicle or not drone_connected:
        return

    if not await set_mode("GUIDED"):
        print("Failed to set GUIDED mode. Cannot takeoff.")
        return

    print(f"Taking off to altitude: {altitude} meters")
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, altitude)
    # Don't sleep. The mavlink_reader will report altitude changes.
    print("Takeoff command sent.")

async def land_vehicle():
    if not vehicle or not drone_connected:
        return

    print("Landing vehicle...")
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0, 0, 0, 0)
    # Don't sleep. The mavlink_reader will report status changes.
    print("Land command sent.")

async def goto_location(latitude, longitude, altitude):
    if not vehicle or not drone_connected:
        return

    if not await set_mode("GUIDED"):
        print("Failed to set GUIDED mode. Cannot go to location.")
        return

    print(f"Moving to Lat: {latitude}, Lon: {longitude}, Alt: {altitude}")
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
    # Don't sleep. The mavlink_reader will report position changes.
    print("Go-to command sent.")

# --- WebSocket Endpoint ---
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    print("WebSocket connected.")
    try:
        # Send initial drone status
        await websocket.send_json(drone_status)

        # Task to continuously read MAVLink messages and send status
        async def mavlink_reader():
            global drone_status
            loop = asyncio.get_event_loop()
            while True:
                try:
                    if vehicle and drone_connected:
                        # Use run_in_executor to avoid blocking the event loop.
                        # Add a timeout to recv_match to prevent it from blocking indefinitely,
                        # which can cause websocket keepalive pings to fail.
                        msg = await loop.run_in_executor(
                            None, functools.partial(vehicle.recv_match, blocking=True, timeout=0.1)
                        )
                        if msg:
                            # Update drone_status based on MAVLink messages
                            if msg.get_type() == 'GLOBAL_POSITION_INT':
                                drone_status["latitude"] = msg.lat / 1e7
                                drone_status["longitude"] = msg.lon / 1e7
                                drone_status["altitude"] = msg.relative_alt / 1000.0 # mm to meters (home-relative, matches GCS)
                                drone_status["heading"] = msg.hdg / 100.0 # centidegrees to degrees
                            elif msg.get_type() == 'HEARTBEAT':
                                # Armed status is derived from system_status
                                is_armed = msg.system_status == mavutil.mavlink.MAV_STATE_ACTIVE
                                drone_status["armed"] = is_armed

                                # Reverse lookup for mode name from mode ID
                                mode_name = "UNKNOWN"
                                for name, mode_id_val in vehicle.mode_mapping().items():
                                    if mode_id_val == msg.custom_mode:
                                        mode_name = name
                                        break
                                drone_status["mode"] = mode_name

                            # Send updated status to frontend ONLY when there's a new message
                            await websocket.send_json(drone_status)
                        else:
                            # No message received within the timeout, yield control briefly
                            await asyncio.sleep(0.01)
                    else:
                        # If not connected, wait a bit before checking again
                        await asyncio.sleep(1)
                except WebSocketDisconnect:
                    print("MAVLink reader: WebSocket disconnected, stopping task.")
                    break  # Exit the loop if the socket is closed
                except Exception as e:
                    print(f"An error occurred in mavlink_reader: {e}")
                    # If any other error occurs, log it and break the loop to be safe
                    break


        reader_task = asyncio.create_task(mavlink_reader())

        while True:
            data = await websocket.receive_text()
            command = json.loads(data)
            print(f"Received command: {command}")

            # Handle commands from frontend
            if command["type"] == "connect":
                if not drone_connected:
                    connect_to_vehicle()
                await websocket.send_json({"type": "status", "message": "Connection attempt initiated."})
            elif command["type"] == "arm":
                await arm_vehicle()
                await websocket.send_json({"type": "status", "message": "Arm command sent."})
            elif command["type"] == "takeoff":
                altitude = float(command["altitude"])
                await takeoff_vehicle(altitude)
                await websocket.send_json({"type": "status", "message": f"Takeoff to {altitude}m command sent."})
            elif command["type"] == "land":
                await land_vehicle()
                await websocket.send_json({"type": "status", "message": "Land command sent."})
            elif command["type"] == "goto":
                lat = float(command["latitude"])
                lon = float(command["longitude"])
                alt = float(command["altitude"])
                await goto_location(lat, lon, alt)
                await websocket.send_json({"type": "status", "message": f"GoTo {lat},{lon},{alt} command sent."})
            elif command["type"] == "mode":
                mode_name = command["mode_name"].upper()
                await set_mode(mode_name)
                await websocket.send_json({"type": "status", "message": f"Mode change to {mode_name} command sent."})

    except WebSocketDisconnect:
        print("WebSocket disconnected.")
    except Exception as e:
        print(f"WebSocket error: {e}")
    finally:
        if 'reader_task' in locals() and not reader_task.done():
            reader_task.cancel()

# --- HTTP Endpoint for Frontend ---
@app.get("/")
async def get_frontend():
    # Serve the index.html file from the frontend directory
    with open("../frontend/index.html", "r", encoding="utf-8") as f:
        return HTMLResponse(content=f.read())

# --- Startup Event ---
@app.on_event("startup")
async def startup_event():
    # Attempt to connect to the drone on startup
    # For a real application, this might be triggered by a user action
    # connect_to_vehicle() # Don't auto-connect, let frontend trigger it
    pass