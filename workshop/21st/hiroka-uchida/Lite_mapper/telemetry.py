"""
telemetry.py
ドローンからのテレメトリ受信・保持を担当するモジュール
バックグラウンドスレッドでMAVLinkメッセージを継続受信する
"""

import threading
import time
import math
import logging

logger = logging.getLogger(__name__)


class TelemetryData:
    """
    最新のテレメトリ状態を保持するスレッドセーフなデータコンテナ
    """

    def __init__(self):
        self._lock = threading.Lock()

        self._data = {
            "connected": False,
            "latitude": None,
            "longitude": None,
            "altitude": None,
            "relative_altitude": None,
            "flight_mode": None,
            "armed": False,
            "heading": None,
            "groundspeed": None,
            "battery_voltage": None,

            # 追加
            "roll": None,
            "pitch": None,
            "yaw": None,

            "last_update": None,
        }

    def update(self, **kwargs):

        with self._lock:

            self._data.update(kwargs)

            self._data["last_update"] = time.time()

    def get(self):

        with self._lock:

            return dict(self._data)

    def set_connected(self, connected: bool):

        with self._lock:

            self._data["connected"] = connected

            if not connected:

                self._data["armed"] = False
                self._data["flight_mode"] = None


class TelemetryWorker:
    """
    MAVLink接続からテレメトリを継続的に取得するバックグラウンドワーカー
    """

    def __init__(
        self,
        drone_connection,
        telemetry_data: TelemetryData,
        interval: float = 1.0,
    ):
        self.drone_connection = drone_connection
        self.telemetry_data = telemetry_data
        self.interval = interval
        self._stop_event = threading.Event()
        self._thread = None

    def start(self):

        if self._thread and self._thread.is_alive():

            logger.warning(
                "TelemetryWorker is already running"
            )

            return

        self._stop_event.clear()

        self._thread = threading.Thread(
            target=self._run,
            daemon=True,
        )

        self._thread.start()

        logger.info(
            "TelemetryWorker started"
        )

    def stop(self):

        self._stop_event.set()

        if self._thread:
            self._thread.join(timeout=3)

        logger.info(
            "TelemetryWorker stopped"
        )

    def _run(self):

        while not self._stop_event.is_set():

            try:

                self._poll_once()

            except Exception as e:

                logger.error(
                    f"Telemetry polling error: {e}"
                )

                self.telemetry_data.set_connected(
                    False
                )

            time.sleep(
                self.interval
            )

    def _poll_once(self):

        master = self.drone_connection.master

        if master is None:

            self.telemetry_data.set_connected(
                False
            )

            return

        messages = {}

        deadline = time.time() + 0.8

        while time.time() < deadline:

            msg = master.recv_match(
                blocking=False
            )

            if msg is None:

                time.sleep(0.02)

                continue

            msg_type = msg.get_type()

            messages[msg_type] = msg

        update_kwargs = {
            "connected": True
        }

        # -------------------------
        # 位置情報
        # -------------------------

        if "GLOBAL_POSITION_INT" in messages:

            gpi = messages[
                "GLOBAL_POSITION_INT"
            ]

            update_kwargs["latitude"] = (
                gpi.lat / 1e7
            )

            update_kwargs["longitude"] = (
                gpi.lon / 1e7
            )

            update_kwargs["altitude"] = (
                gpi.alt / 1000.0
            )

            update_kwargs[
                "relative_altitude"
            ] = (
                gpi.relative_alt / 1000.0
            )

            update_kwargs["heading"] = (
                gpi.hdg / 100.0
            )

        # -------------------------
        # フライトモード
        # -------------------------

        if "HEARTBEAT" in messages:

            try:

                update_kwargs[
                    "flight_mode"
                ] = master.flightmode

                update_kwargs[
                    "armed"
                ] = master.motors_armed()

            except Exception as e:

                logger.error(
                    f"Heartbeat error: {e}"
                )

        # -------------------------
        # 対地速度
        # -------------------------

        if "VFR_HUD" in messages:

            vfr = messages["VFR_HUD"]

            update_kwargs[
                "groundspeed"
            ] = vfr.groundspeed

        # -------------------------
        # バッテリー
        # -------------------------

        if "SYS_STATUS" in messages:

            sys_status = messages[
                "SYS_STATUS"
            ]

            update_kwargs[
                "battery_voltage"
            ] = (
                sys_status.voltage_battery
                / 1000.0
            )

        # -------------------------
        # 姿勢情報
        # -------------------------

        if "ATTITUDE" in messages:

            att = messages["ATTITUDE"]

            update_kwargs["roll"] = round(
                math.degrees(att.roll),
                2
            )

            update_kwargs["pitch"] = round(
                math.degrees(att.pitch),
                2
            )

            update_kwargs["yaw"] = round(
                math.degrees(att.yaw),
                2
            )

        self.telemetry_data.update(
            **update_kwargs
        )