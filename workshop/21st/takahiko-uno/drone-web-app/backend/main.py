"""ドローン Web 制御アプリ バックエンド (FastAPI + WebSocket + pymavlink)。

- `/`            : フロントエンドの index.html を返す
- `/static`      : frontend ディレクトリを配信
- `/ws`          : WebSocket。コマンド受信と状態配信を行う
- 起動ポート      : 9999

MAVLink 受信は blocking な recv_match を executor(別スレッド) 上で実行し、
イベントループ・WebSocket をブロックしないようにしている。
"""

import asyncio
import json
import logging
import os
import time
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response
from fastapi.staticfiles import StaticFiles
from pymavlink import mavutil

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("drone-web-app")

BASE_DIR = Path(__file__).resolve().parent.parent
FRONTEND_DIR = BASE_DIR / "frontend"

# 接続先は環境変数化。BlueOS(bridge)では localhost では Router に届かないため、
# 既定は host.docker.internal 宛の udpout(client)。ローカル確認は
#   MAV_ENDPOINT=tcp:127.0.0.1:5762
# で上書きする。
CONNECTION_STRING = os.environ.get("MAV_ENDPOINT", "udpout:host.docker.internal:14550")
SELECTABLE_MODES = ["GUIDED", "AUTO", "RTL", "LOITER", "STABILIZE"]
GUIDED_WAIT_SEC = 5.0

# ---------------------------------------------------------------------------
# 状態（全 WebSocket クライアントで共有する単一の状態オブジェクト）
# ---------------------------------------------------------------------------
state = {
    "connected": False,
    "armed": False,
    "mode": "UNKNOWN",
    "latitude": 0.0,
    "longitude": 0.0,
    "altitude": 0.0,
    "heading": 0,
}


class ConnectionManager:
    """接続中の WebSocket クライアントを管理し、状態をブロードキャストする。"""

    def __init__(self):
        self.clients: set[WebSocket] = set()

    async def connect(self, ws: WebSocket):
        await ws.accept()
        self.clients.add(ws)

    def disconnect(self, ws: WebSocket):
        self.clients.discard(ws)

    async def broadcast(self, message: dict):
        dead = []
        for ws in list(self.clients):
            try:
                await ws.send_json(message)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.clients.discard(ws)


manager = ConnectionManager()


async def broadcast_state():
    """現在の状態を state メッセージとして全クライアントへ送る。"""
    await manager.broadcast({"type": "state", "state": state})


class DroneManager:
    """MAVLink 接続・コマンド送信・テレメトリー受信をまとめて扱う。"""

    def __init__(self):
        self.vehicle = None
        self.target_system = None
        self.target_component = None
        # 機体タイプから明示生成したモードマップ（name -> custom_mode）。
        self.mode_map: dict[str, int] = {}
        self._recv_task: asyncio.Task | None = None
        self._connect_lock = asyncio.Lock()

    # -- 汎用: blocking な処理を executor で実行 -----------------------------
    async def _run(self, fn, *args):
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(None, fn, *args)

    def _tgt_sys(self):
        return self.target_system if self.target_system is not None else self.vehicle.target_system

    def _tgt_comp(self):
        return self.target_component if self.target_component is not None else self.vehicle.target_component

    # -- 接続 ---------------------------------------------------------------
    async def connect(self) -> tuple[bool, str]:
        async with self._connect_lock:
            if self.vehicle is not None:
                return True, "既に接続済みです"
            try:
                await self._run(self._connect_blocking)
            except Exception as e:  # noqa: BLE001
                logger.exception("MAVLink 接続に失敗しました")
                self.vehicle = None
                return False, f"接続に失敗しました: {e}"

            state["connected"] = True
            self._recv_task = asyncio.create_task(self._receive_loop())
            await broadcast_state()
            logger.info("MAVLink 接続完了 (%s)", CONNECTION_STRING)
            return True, "機体に接続しました"

    def _connect_blocking(self):
        """別スレッドで実行される blocking な接続処理。"""
        self.vehicle = mavutil.mavlink_connection(CONNECTION_STRING)
        # GCS として HEARTBEAT を送る（Router に自分を認識させる）。
        self.vehicle.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0,
        )
        # Router 経由では GCS 等の HEARTBEAT も混ざる。autopilot(非INVALID) を特定する。
        hb = None
        deadline = time.time() + 30
        while time.time() < deadline:
            msg = self.vehicle.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if msg and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                hb = msg
                break
        if hb is None:
            raise RuntimeError("オートパイロットの HEARTBEAT を検出できませんでした")

        self.target_system = hb.get_srcSystem()
        self.target_component = hb.get_srcComponent()
        self.vehicle.target_system = self.target_system
        self.vehicle.target_component = self.target_component
        # 機体タイプから明示生成する。mode_mapping() は直近 HEARTBEAT を見て
        # Plane/Copter を誤り、GUIDED 等の番号がずれる（Copter GUIDED=4 / Plane=15）。
        self.mode_map = mavutil.mode_mapping_byname(hb.type) or {}
        self._request_streams()

    def _request_streams(self):
        """位置系を含むデータストリームの送信を機体へ要求する。"""
        for stream in (
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
        ):
            self.vehicle.mav.request_data_stream_send(
                self.vehicle.target_system,
                self.vehicle.target_component,
                stream,
                4,  # 4 Hz
                1,  # start
            )

    # -- 受信ループ ---------------------------------------------------------
    def _recv_once(self):
        """1 メッセージだけ blocking 受信する。例外は握りつぶしてログに残す。"""
        try:
            return self.vehicle.recv_match(blocking=True, timeout=0.1)
        except Exception as e:  # noqa: BLE001
            logger.warning("recv_match で例外: %s", e)
            return None

    async def _receive_loop(self):
        loop = asyncio.get_event_loop()
        while self.vehicle is not None:
            # blocking 受信は executor 上で。timeout=0.1 なので無限ブロックしない。
            msg = await loop.run_in_executor(None, self._recv_once)
            if msg is None:
                continue
            try:
                changed = self._handle_message(msg)
            except Exception:  # noqa: BLE001
                logger.exception("受信メッセージの処理でエラー")
                continue
            if changed:
                await broadcast_state()

    def _mode_name(self, custom_mode) -> str:
        if self.mode_map:
            inverse = {v: k for k, v in self.mode_map.items()}
            return inverse.get(custom_mode, str(custom_mode))
        return "UNKNOWN"

    def _handle_message(self, msg) -> bool:
        """受信メッセージで状態を更新。更新があれば True を返す。"""
        # 自機(接続時に確定した target sys/comp)以外は無視する。Router 経由では
        # GCS 等の HEARTBEAT も届き、mode/armed が点滅する原因になる。
        if self.target_system is not None and (
            msg.get_srcSystem() != self.target_system
            or msg.get_srcComponent() != self.target_component
        ):
            return False

        mtype = msg.get_type()

        if mtype == "GLOBAL_POSITION_INT":
            state["latitude"] = msg.lat / 1e7
            state["longitude"] = msg.lon / 1e7
            # relative_alt(mm) を優先。無ければ alt(mm)。
            if hasattr(msg, "relative_alt"):
                state["altitude"] = msg.relative_alt / 1000
            else:
                state["altitude"] = msg.alt / 1000
            if msg.hdg != 65535:  # 65535 は不明値
                state["heading"] = msg.hdg / 100
            return True

        if mtype == "HEARTBEAT":
            # ARM は system_status ではなく base_mode の SAFETY_ARMED フラグで判定する。
            state["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            state["mode"] = self._mode_name(msg.custom_mode)
            return True

        return False

    # -- コマンド -----------------------------------------------------------
    async def execute(self, ctype: str, cmd: dict) -> tuple[bool, str]:
        if ctype == "connect":
            return await self.connect()

        # connect 以外は未接続だと実行できない（落ちないように弾く）。
        if self.vehicle is None:
            return False, "未接続です。先に「接続」してください"

        if ctype == "arm":
            return await self.arm(True)
        if ctype == "disarm":
            return await self.arm(False)
        if ctype == "takeoff":
            return await self.takeoff(float(cmd.get("altitude", 0) or 0))
        if ctype == "land":
            return await self.land()
        if ctype == "goto":
            return await self.goto(
                float(cmd.get("latitude")),
                float(cmd.get("longitude")),
                float(cmd.get("altitude", 0) or 0),
            )
        if ctype == "mode":
            return await self.set_mode(cmd.get("mode"))
        return False, f"未知のコマンドです: {ctype}"

    async def ensure_guided(self) -> bool:
        """takeoff / goto の前に GUIDED への切替を試みる（最大 5 秒待機）。"""
        if state["mode"] == "GUIDED":
            return True
        await self._run(self._set_mode_blocking, "GUIDED")
        steps = int(GUIDED_WAIT_SEC / 0.1)
        for _ in range(steps):
            if state["mode"] == "GUIDED":
                return True
            await asyncio.sleep(0.1)
        return state["mode"] == "GUIDED"

    def _set_mode_blocking(self, mode_name: str) -> bool:
        if not self.mode_map or mode_name not in self.mode_map:
            return False
        # command_long の DO_SET_MODE は ArduPilot で反映保証がないため set_mode() を使う。
        self.vehicle.set_mode(self.mode_map[mode_name])
        return True

    async def set_mode(self, mode_name) -> tuple[bool, str]:
        if mode_name not in SELECTABLE_MODES:
            return False, f"未対応のモードです: {mode_name}"
        applied = await self._run(self._set_mode_blocking, mode_name)
        if not applied:
            return False, f"モード {mode_name} へ切替できませんでした"
        return True, f"モードを {mode_name} に設定しました"

    def _send_arm(self, value: int):
        self.vehicle.mav.command_long_send(
            self._tgt_sys(), self._tgt_comp(),
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            value, 0, 0, 0, 0, 0, 0,
        )

    async def arm(self, arm_it: bool) -> tuple[bool, str]:
        await self._run(self._send_arm, 1 if arm_it else 0)
        msg = "アームコマンドを送信しました" if arm_it else "ディスアームコマンドを送信しました"
        return True, msg

    def _send_takeoff(self, altitude: float):
        self.vehicle.mav.command_long_send(
            self._tgt_sys(), self._tgt_comp(),
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, altitude,  # param7 = 目標高度
        )

    async def takeoff(self, altitude: float) -> tuple[bool, str]:
        if not await self.ensure_guided():
            return False, "GUIDED へ切替できなかったため離陸を中止しました"
        await self._run(self._send_takeoff, altitude)
        return True, f"離陸コマンドを送信しました（目標高度 {altitude} m）"

    def _send_land(self):
        self.vehicle.mav.command_long_send(
            self._tgt_sys(), self._tgt_comp(),
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
            0, 0, 0, 0, 0, 0, 0,
        )

    async def land(self) -> tuple[bool, str]:
        await self._run(self._send_land)
        return True, "着陸コマンドを送信しました"

    def _send_goto(self, lat: float, lon: float, alt: float):
        self.vehicle.mav.set_position_target_global_int_send(
            0,  # time_boot_ms
            self._tgt_sys(), self._tgt_comp(),
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,  # 位置のみ有効（速度・加速度・yaw を無効化）
            int(lat * 1e7), int(lon * 1e7), float(alt),
            0, 0, 0,  # vx, vy, vz
            0, 0, 0,  # afx, afy, afz
            0, 0,     # yaw, yaw_rate
        )

    async def goto(self, lat: float, lon: float, alt: float) -> tuple[bool, str]:
        if not await self.ensure_guided():
            return False, "GUIDED へ切替できなかったため移動を中止しました"
        await self._run(self._send_goto, lat, lon, alt)
        return True, f"移動コマンドを送信しました（{lat:.6f}, {lon:.6f}, {alt} m）"

    async def shutdown(self):
        vehicle = self.vehicle
        self.vehicle = None
        self.target_system = None
        self.target_component = None
        self.mode_map = {}
        if self._recv_task is not None:
            self._recv_task.cancel()
            try:
                await self._recv_task
            except (asyncio.CancelledError, Exception):  # noqa: BLE001
                pass
            self._recv_task = None
        if vehicle is not None:
            try:
                vehicle.close()
            except Exception:  # noqa: BLE001
                pass


drone = DroneManager()

app = FastAPI(title="Drone Web App")
app.mount("/static", StaticFiles(directory=str(FRONTEND_DIR)), name="static")


@app.get("/")
async def index():
    return FileResponse(str(FRONTEND_DIR / "index.html"))


@app.get("/register_service")
async def register_service():
    """BlueOS Extension Helper 向けのサービス登録情報。

    WebSocket は BlueOS のプロキシに非対応のため avoid_iframes=True とし、
    新規ウィンドウで http://<IP>:9999/ を直接開かせる。
    """
    return {
        "name": "Drone Web App",
        "description": "pymavlink + FastAPI + WebSocket でドローンを制御する Web アプリ",
        "icon": "mdi-drone",
        "company": "",
        "version": "1.0.0",
        "webpage": "",
        "api": "/docs",
        "avoid_iframes": True,
    }


@app.get("/favicon.ico")
async def favicon():
    # HTML 側でインライン SVG を指定済みだが、キャッシュ等で /favicon.ico を
    # 要求するブラウザ向けのフォールバック（204 で 404 ログを防ぐ）。
    return Response(status_code=204)


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await manager.connect(ws)
    # 接続直後に現在の状態を即時送信する。
    try:
        await ws.send_json({"type": "state", "state": state})
        while True:
            raw = await ws.receive_text()
            await handle_command(raw, ws)
    except WebSocketDisconnect:
        manager.disconnect(ws)
    except Exception:  # noqa: BLE001
        logger.exception("WebSocket 処理でエラー")
        manager.disconnect(ws)


async def handle_command(raw: str, ws: WebSocket):
    try:
        cmd = json.loads(raw)
    except (json.JSONDecodeError, TypeError):
        await ws.send_json({"type": "status", "ok": False, "message": "不正なコマンド形式です"})
        return

    ctype = cmd.get("type")
    try:
        ok, message = await drone.execute(ctype, cmd)
    except Exception as e:  # noqa: BLE001
        logger.exception("コマンド実行でエラー: %s", ctype)
        ok, message = False, f"コマンド実行でエラーが発生しました: {e}"
    await ws.send_json({"type": "status", "ok": ok, "message": message})


@app.on_event("shutdown")
async def on_shutdown():
    await drone.shutdown()


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=9999)
