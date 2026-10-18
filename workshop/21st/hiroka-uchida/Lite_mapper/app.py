import logging
import atexit
from datetime import datetime

from flask import Flask, jsonify, request, render_template

from drone_connection import DroneConnection
from telemetry import TelemetryData, TelemetryWorker


logging.basicConfig(level=logging.INFO)

app = Flask(__name__)

drone_connection = DroneConnection()

telemetry_data = TelemetryData()

telemetry_worker = TelemetryWorker(
    drone_connection,
    telemetry_data,
    interval=1.0,
)

saved_points = []

@app.route("/")
def index():
    return render_template("index.html")


@app.route("/connect", methods=["POST"])
def connect():

    body = request.get_json(silent=True) or {}

    connection_string = body.get(
        "connection_string",
        "tcp:127.0.0.1:5760",
    )

    if drone_connection.is_connected():
        return jsonify({
            "status": "already_connected"
        })

    try:
        drone_connection.connect(connection_string)

    except Exception as e:

        logging.exception("Connect Error")

        return jsonify({
            "status": "error",
            "message": str(e)
        }), 500

    telemetry_data.set_connected(True)

    telemetry_worker.start()

    return jsonify({
        "status": "connected"
    })


@app.route("/disconnect", methods=["POST"])
def disconnect():

    telemetry_worker.stop()

    drone_connection.disconnect()

    telemetry_data.set_connected(False)

    return jsonify({
        "status": "disconnected"
    })


@app.route("/telemetry")
def telemetry():

    data = telemetry_data.get()

    print(data)

    return jsonify(data)


@app.route("/arm", methods=["POST"])
def arm():

    if not drone_connection.is_connected():
        return jsonify({
            "status": "error",
            "message": "not connected"
        }), 400

    body = request.get_json(
        silent=True
    ) or {}

    force = bool(
        body.get("force", False)
    )

    try:

        success = drone_connection.arm(
            force=force
        )

    except Exception as e:

        logging.exception("ARM Error")

        return jsonify({
            "status": "error",
            "message": str(e)
        }), 500

    if success:
        return jsonify({
            "status": "armed"
        })

    return jsonify({
        "status": "failed"
    }), 400


@app.route("/disarm", methods=["POST"])
def disarm():

    if not drone_connection.is_connected():
        return jsonify({
            "status": "error",
            "message": "not connected"
        }), 400

    body = request.get_json(
        silent=True
    ) or {}

    force = bool(
        body.get("force", False)
    )

    try:

        success = drone_connection.disarm(
            force=force
        )

    except Exception as e:

        logging.exception(
            "DISARM Error"
        )

        return jsonify({
            "status": "error",
            "message": str(e)
        }), 500

    if success:
        return jsonify({
            "status": "disarmed"
        })

    return jsonify({
        "status": "failed"
    }), 400


@app.route("/mode", methods=["POST"])
def set_mode():

    logging.info(
        "MODE API CALLED"
    )

    if not drone_connection.is_connected():
        return jsonify({
            "status": "error",
            "message": "not connected"
        }), 400

    body = request.get_json(
        silent=True
    ) or {}

    mode_name = body.get("mode")

    if not mode_name:
        return jsonify({
            "status": "error",
            "message": "mode is required"
        }), 400

    try:

        success = drone_connection.set_mode(
            mode_name
        )

    except Exception as e:

        logging.exception(
            "MODE Error"
        )

        return jsonify({
            "status": "error",
            "message": str(e)
        }), 500

    if success:
        return jsonify({
            "status": "mode_changed",
            "mode": mode_name,
        })

    return jsonify({
        "status": "failed"
    }), 400


@app.route("/takeoff", methods=["POST"])
def takeoff():

    if not drone_connection.is_connected():
        return jsonify({
            "status": "error",
            "message": "not connected"
        }), 400

    body = request.get_json(
        silent=True
    ) or {}

    altitude = float(
        body.get("altitude", 5)
    )

    try:

        success = drone_connection.takeoff(
            altitude
        )

    except Exception as e:

        return jsonify({
            "status": "error",
            "message": str(e)
        }), 500

    if success:

        return jsonify({
            "status": "takeoff_started",
            "altitude": altitude,
        })

    return jsonify({
        "status": "failed"
    }), 400


@app.route("/land", methods=["POST"])
def land():

    if not drone_connection.is_connected():
        return jsonify({
            "status": "error",
            "message": "not connected"
        }), 400

    try:

        success = drone_connection.land()

    except Exception as e:

        return jsonify({
            "status": "error",
            "message": str(e)
        }), 500

    if success:
        return jsonify({
            "status": "landing"
        })

    return jsonify({
        "status": "failed"
    }), 400


@app.route("/goto", methods=["POST"])
def goto():

    if not drone_connection.is_connected():
        return jsonify({
            "status": "error",
            "message": "not connected"
        }), 400

    body = request.get_json(
        silent=True
    ) or {}

    latitude = float(
        body["latitude"]
    )

    longitude = float(
        body["longitude"]
    )

    altitude = float(
        body.get("altitude", 10)
    )

    try:

        success = drone_connection.goto_location(
            latitude,
            longitude,
            altitude,
        )

    except Exception as e:

        return jsonify({
            "status": "error",
            "message": str(e)
        }), 500

    if success:

        return jsonify({
            "status": "moving",
            "latitude": latitude,
            "longitude": longitude,
            "altitude": altitude,
        })

    return jsonify({
        "status": "failed"
    }), 400

@app.route("/save_point", methods=["POST"])
def save_point():

    telemetry = telemetry_data.get()

    point = {
        "timestamp": datetime.now().strftime(
            "%Y-%m-%d %H:%M:%S"
        ),

        "latitude": telemetry.get("latitude"),
        "longitude": telemetry.get("longitude"),
        "altitude": telemetry.get(
            "relative_altitude"
        ),

        "roll": telemetry.get("roll"),
        "pitch": telemetry.get("pitch"),
        "yaw": telemetry.get("yaw"),

        "heading": telemetry.get(
            "heading"
        ),

        "groundspeed": telemetry.get(
            "groundspeed"
        ),

        "flight_mode": telemetry.get(
            "flight_mode"
        ),

        "armed": telemetry.get(
            "armed"
        ),
    }

    saved_points.append(point)

    return jsonify({
        "status": "saved",
        "point": point
    })

@app.route("/saved_points")
def saved_points_api():

    return jsonify(saved_points)

def emergency_rtl():

    if drone_connection.is_connected():

        try:

            print(
                "Returning to launch before shutdown..."
            )

            drone_connection.rtl()

        except Exception as e:

            print(e)


atexit.register(
    emergency_rtl
)


if __name__ == "__main__":

    app.run(
        host="0.0.0.0",
        port=5000,
        debug=True,
        use_reloader=False,
    )

