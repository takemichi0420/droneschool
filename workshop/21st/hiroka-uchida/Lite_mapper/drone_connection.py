"""
drone_connection.py
MAVLink接続の確立、ARM/DISARM、モード変更などの機体制御を担当するモジュール
"""

import logging

from pymavlink import mavutil

logger = logging.getLogger(__name__)


class DroneConnection:
    """
    MAVLink接続を保持し、機体制御コマンドを送信するクラス
    """

    def __init__(self):
        self.master = None
        self.connection_string = None

    def connect(
        self,
        connection_string: str,
        timeout: float = 30,
    ):
        """
        SITL/MAVProxy/実機へ接続し、ハートビートを待つ
        """

        logger.info(
            f"Connecting to {connection_string}"
        )

        self.connection_string = connection_string

        self.master = mavutil.mavlink_connection(
            connection_string
        )

        logger.info(
            "Waiting heartbeat..."
        )

        self.master.wait_heartbeat(
            timeout=timeout
        )

        logger.info(
            f"Heartbeat from system="
            f"{self.master.target_system} "
            f"component="
            f"{self.master.target_component}"
        )

        # テレメトリ要求
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10,
            1,
        )

        logger.info(
            "Requested MAVLink data streams"
        )

        return self.master

    def disconnect(self):

        if self.master is not None:

            try:

                self.master.close()

            except Exception as e:

                logger.warning(
                    f"Error while closing connection: {e}"
                )

            finally:

                self.master = None

    def is_connected(self) -> bool:

        return self.master is not None

    def get_mode_string(
        self,
        custom_mode: int
    ) -> str:

        if self.master is None:
            raise RuntimeError(
                "Not connected"
            )

        mode_mapping = (
            self.master.mode_mapping()
        )

        if mode_mapping is None:
            return str(custom_mode)

        for name, value in (
            mode_mapping.items()
        ):

            if value == custom_mode:
                return name

        return str(custom_mode)

    def set_mode(
        self,
        mode_name: str
    ) -> bool:
        """
        フライトモード変更
        """

        if self.master is None:
            raise RuntimeError(
                "Not connected"
            )

        mode_mapping = (
            self.master.mode_mapping()
        )

        if mode_name not in mode_mapping:

            logger.error(
                f"Unknown mode: {mode_name}"
            )

            return False

        mode_id = mode_mapping[
            mode_name
        ]

        logger.info(
            f"Changing mode to "
            f"{mode_name}"
        )

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
            0,
            0,
            0,
            0,
            0,
        )

        return True

    def arm(
        self,
        force: bool = False
    ) -> bool:

        if self.master is None:
            raise RuntimeError(
                "Not connected"
            )

        logger.info(
            "Arming vehicle..."
        )

        self.master.arducopter_arm()

        self.master.motors_armed_wait()

        logger.info(
            "ARMED"
        )

        return True

    def disarm(
        self,
        force: bool = False
    ) -> bool:

        if self.master is None:
            raise RuntimeError(
                "Not connected"
            )

        logger.info(
            "Disarming vehicle..."
        )

        self.master.arducopter_disarm()

        self.master.motors_disarmed_wait()

        logger.info(
            "DISARMED"
        )

        return True

    def land(self) -> bool:

        if self.master is None:
            raise RuntimeError(
                "Not connected"
            )

        success = self.set_mode(
            "LAND"
        )

        if success:

            logger.info(
                "LAND mode set"
            )

        return success

    def rtl(self) -> bool:

        if self.master is None:
            raise RuntimeError(
                "Not connected"
            )

        success = self.set_mode(
            "RTL"
        )

        if success:

            logger.info(
                "RTL mode set"
            )

        return success

    def takeoff(
        self,
        altitude: float = 5.0
    ) -> bool:

        if self.master is None:
            raise RuntimeError(
                "Not connected"
            )

        self.set_mode(
            "GUIDED"
        )

        self.master.arducopter_arm()

        self.master.motors_armed_wait()

        logger.info(
            "ARMED"
        )

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
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

        logger.info(
            f"Takeoff command sent: "
            f"{altitude}m"
        )

        return True

    def goto_location(
        self,
        latitude: float,
        longitude: float,
        altitude: float = 10.0,
    ) -> bool:

        if self.master is None:
            raise RuntimeError(
                "Not connected"
            )

        self.set_mode(
            "GUIDED"
        )

        self.master.mav.set_position_target_global_int_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b110111111000,
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

        logger.info(
            f"GOTO lat={latitude} "
            f"lon={longitude} "
            f"alt={altitude}"
        )

        return True