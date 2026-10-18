"""
config.py
アプリケーション設定・定数管理
"""

import os


class Config:

    # Flask設定
    SECRET_KEY = os.environ.get(
        "SECRET_KEY",
        "lite-mapper-dev-key"
    )

    DEBUG = (
        os.environ.get(
            "FLASK_DEBUG",
            "True"
        ) == "True"
    )

    # Flaskサーバー設定
    HOST = "0.0.0.0"
    PORT = 5000

    # ==========================
    # MAVLink接続設定
    # ==========================

    # SITL用
    SITL_CONNECTION_STRING = (
        "tcp:127.0.0.1:5762"
    )

    # BlueOS用
    BLUEOS_CONNECTION_STRING = (
        "udpin:0.0.0.0:14550"
    )

    # 現在使用する接続先
    CONNECTION_STRING = (
        SITL_CONNECTION_STRING
    )

    # 接続タイムアウト
    CONNECTION_TIMEOUT = 10

    # ==========================
    # テレメトリ
    # ==========================

    TELEMETRY_UPDATE_INTERVAL = 1.0

    # ==========================
    # フライト設定
    # ==========================

    DEFAULT_TAKEOFF_ALTITUDE = 5

    SUPPORTED_MODES = [
        "GUIDED",
        "RTL",
        "LAND"
    ]

    # ==========================
    # ログ設定
    # ==========================

    LOG_DIR = os.path.join(
        os.path.dirname(__file__),
        "logs"
    )

    LOG_FILE = os.path.join(
        LOG_DIR,
        "app.log"
    )

    LOG_LEVEL = "INFO"


class MAVLinkConstants:
    """
    ArduPilot Copter
    モードマッピング
    """

    COPTER_MODE_MAP = {
        "STABILIZE": 0,
        "ACRO": 1,
        "ALT_HOLD": 2,
        "AUTO": 3,
        "GUIDED": 4,
        "LOITER": 5,
        "RTL": 6,
        "CIRCLE": 7,
        "LAND": 9,
        "DRIFT": 11,
        "SPORT": 13,
        "FLIP": 14,
        "AUTOTUNE": 15,
        "POSHOLD": 16,
        "BRAKE": 17,
        "THROW": 18,
        "AVOID_ADSB": 19,
        "GUIDED_NOGPS": 20,
        "SMART_RTL": 21,
    }