""""
drone_control.py
pymavlinkを用いたドローン制御ロジック（接続・モード変更・ARM・離陸・移動・RTL）
"""

import time
import logging
from pymavlink import mavutil

from config import MAVLinkConstants

logger = logging.getLogger(__name__)


class DroneConnectionError(Exception):
    pass


class DroneControl:
    """
    ドローンとのMAVLink通信・制御コマンド送信を担当するクラス
    """

    def __init__(self):
        self.master = None
        self.connection_string = None
        self._mode_map_inverse = {v: k for k, v in MAVLinkConstants.COPTER_MODE_MAP.items()}

    # ------------------------------------------------------------------
    # 接続
    # ------------------------------------------------------------------
    def connect(self, connection_string: str, timeout: int = 10) -> bool:
        """
        指定された接続文字列でドローンに接続する
        例: "tcp:127.0.0.1:5762", "udp:127.0.0.1:14550"
        """
        try:
            logger.info(f"Connecting to {connection_string} ...")
            self.master = mavutil.mavlink_connection(connection_string)

            heartbeat = self.master.wait_heartbeat(timeout=timeout)
            if heartbeat is None:
                raise DroneConnectionError("Heartbeat not received. Connection failed.")

            self.connection_string = connection_string
            logger.info(
                f"Connected. system={self.master.target_system} "
                f"component={self.master.target_component}"
            )
            return True

        except Exception as e:
            logger.error(f"Connection failed: {e}")
            self.master = None
            raise DroneConnectionError(str(e))

    def disconnect(self):
        if self.master:
            try:
                self.master.close()
            except Exception as e:
                logger.warning(f"Error on disconnect: {e}")
            finally:
                self.master = None
                self.connection_string = None
        logger.info("Disconnected")

    def is_connected(self) -> bool:
        return self.master is not None

    def get_mode_string(self, custom_mode: int) -> str:
        return self._mode_map_inverse.get(custom_mode, f"UNKNOWN({custom_mode})")

    # ------------------------------------------------------------------
    # モード変更
    # ------------------------------------------------------------------
    def set_mode(self, mode_name: str) -> bool:

    if self.master is None:
        raise RuntimeError("Not connected")

    if mode_name not in self.master.mode_mapping():
        raise ValueError(f"Unknown mode: {mode_name}")

    mode_id = self.master.mode_mapping()[mode_name]

    logger.info(f"Changing mode to {mode_name}")

    self.master.mav.command_long_send(
        self.master.target_system,
        self.master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
        0, 0, 0, 0, 0
    )

    # モード変更完了待ち
    for _ in range(50):

        self.master.recv_msg()

        if self.master.flightmode == mode_name:
            logger.info(f"Mode changed to {mode_name}")
            return True

    logger.error(
        f"Mode change failed. Current mode={self.master.flightmode}"
    )

    return False

    def _wait_for_mode(self, target_mode: str, timeout: float = 5) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if msg is None:
                continue
            current_mode = self.get_mode_string(msg.custom_mode)
            if current_mode == target_mode:
                return True
        return False

    # ------------------------------------------------------------------
    # ARM / DISARM
    # ------------------------------------------------------------------
    def arm(self) -> bool:
        self._require_connection()
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,  # 1 = arm
            0, 0, 0, 0, 0, 0,
        )
        result = self._wait_for_command_ack(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        )
        if result:
            logger.info("Armed")
        else:
            logger.warning("Arm command not acknowledged")
        return result

    def disarm(self) -> bool:
        self._require_connection()
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,  # 0 = disarm
            0, 0, 0, 0, 0, 0,
        )
        result = self._wait_for_command_ack(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        )
        if result:
            logger.info("Disarmed")
        else:
            logger.warning("Disarm command not acknowledged")
        return result

    def _wait_for_command_ack(self, command_id: int, timeout: float = 5) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.master.recv_match(type="COMMAND_ACK", blocking=True, timeout=1)
            if msg is None:
                continue
            if msg.command == command_id:
                return msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
        return False

    # ------------------------------------------------------------------
    # 離陸
    # ------------------------------------------------------------------
    def takeoff(self, altitude: float) -> bool:
        """
        GUIDED -> ARM -> TAKEOFF の順で実行する
        """
        self._require_connection()

        if not self.set_mode("GUIDED"):
            raise RuntimeError("Failed to switch to GUIDED mode")

        if not self.arm():
            raise RuntimeError("Failed to arm")

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            altitude,
        )

        result = self._wait_for_command_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
        if result:
            logger.info(f"Takeoff command accepted (target altitude={altitude}m)")
        else:
            logger.warning("Takeoff command not acknowledged")
        return result

    # ------------------------------------------------------------------
    # 移動（GPS座標指定）
    # ------------------------------------------------------------------
    def goto(self, latitude: float, longitude: float, altitude: float) -> bool:
        """
        指定したGPS座標・高度へ移動する（GUIDEDモード時のみ有効）
        SET_POSITION_TARGET_GLOBAL_INT を使用
        """
        self._require_connection()

        type_mask = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)
        self.master.mav.set_position_target_global_int_send(
            0,  # time_boot_ms
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            type_mask,
            int(latitude * 1e7),
            int(longitude * 1e7),
            altitude,
            0, 0, 0,  # vx, vy, vz
            0, 0, 0,  # afx, afy, afz
            0, 0,      # yaw, yaw_rate
        )
        logger.info(f"Goto command sent: lat={latitude}, lon={longitude}, alt={altitude}")
        return True

    # ------------------------------------------------------------------
    # RTL
    # ------------------------------------------------------------------
    def rtl(self) -> bool:
        self._require_connection()
        result = self.set_mode("RTL")
        if result:
            logger.info("RTL mode activated")
        return result

    # ------------------------------------------------------------------
    # LAND
    # ------------------------------------------------------------------
    def land(self) -> bool:
        self._require_connection()
        result = self.set_mode("LAND")
        if result:
            logger.info("LAND mode activated")
        return result

    # ------------------------------------------------------------------
    # 内部ユーティリティ
    # ------------------------------------------------------------------
    def _require_connection(self):
        if self.master is None:
            raise DroneConnectionError("Drone is not connected")