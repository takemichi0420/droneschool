"""ドローンWeb制御アプリケーション バックエンド.

FastAPI + WebSocket + pymavlink で、ブラウザからドローン(SITL)を制御する。
MAVLink の受信ブロッキングは executor に逃がし、イベントループを止めない。
"""

import asyncio
import logging
import os
import socket
import struct
import time
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from starlette.websockets import WebSocketState
from pymavlink import mavutil

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("drone-web-app")

# 接続先。BlueOS では bridge から Router へ udpout で届ける(既定)。
# ローカル SITL 確認時は MAV_ENDPOINT=tcp:127.0.0.1:5762 で上書きする。
CONNECTION_STRING = os.environ.get(
    "MAV_ENDPOINT", "udpout:host.docker.internal:14550"
)
# 起動ポート
PORT = 9999

FRONTEND_DIR = Path(__file__).resolve().parent.parent / "frontend"

app = FastAPI(title="Drone Web Control")


class DroneState:
    """機体のリアルタイム状態を保持する。"""

    def __init__(self) -> None:
        self.connected = False
        self.armed = False
        self.mode = "UNKNOWN"
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.heading = 0

    def to_dict(self) -> dict:
        return {
            "connected": self.connected,
            "armed": self.armed,
            "mode": self.mode,
            "latitude": self.latitude,
            "longitude": self.longitude,
            "altitude": self.altitude,
            "heading": self.heading,
        }


class DroneManager:
    """MAVLink 接続とコマンド送信を管理する。"""

    def __init__(self) -> None:
        self.state = DroneState()
        self.vehicle = None
        # 本物のオートパイロットの発生源(HEARTBEAT で確定)
        self.target_system = 1
        self.target_component = 1
        # 機体タイプから明示生成したモードマップ(名前 -> custom_mode)
        self.mode_map: dict[str, int] = {}
        self._recv_task: asyncio.Task | None = None
        self._lock = asyncio.Lock()
        # 機体側イベント(COMMAND_ACK / STATUSTEXT)を UI へ流すためのキュー
        self.events: asyncio.Queue = asyncio.Queue(maxsize=200)
        # 直近の COMMAND_ACK(command id -> result)。ACK 待ち合わせに使う。
        self._acks: dict[int, int] = {}

    def _emit(self, message: str) -> None:
        """機体側メッセージを UI 送信用キューへ入れる(満杯なら破棄)。"""
        try:
            self.events.put_nowait(message)
        except asyncio.QueueFull:
            pass

    # ---- 接続 -------------------------------------------------------------

    async def connect(self) -> str:
        """未接続なら機体へ接続する。接続待ちは executor で行う。"""
        async with self._lock:
            if self.state.connected:
                return "既に接続済みです"
            loop = asyncio.get_running_loop()
            try:
                self.vehicle = await loop.run_in_executor(None, self._blocking_connect)
            except Exception as exc:  # noqa: BLE001
                logger.exception("接続に失敗しました")
                return f"接続に失敗しました: {exc}"

            self.state.connected = True
            self._request_data_streams()
            self._recv_task = asyncio.create_task(self._recv_loop())
            logger.info("MAVLink 接続完了: sys=%s comp=%s",
                        self.target_system, self.target_component)
            return "機体へ接続しました"

    def _blocking_connect(self):
        """ブロッキングな接続処理(executor 上で実行)。

        Router 経由(BlueOS)では GCS の HEARTBEAT も混ざるため、
        本物のオートパイロット(非GCS)を特定してターゲットに据える。
        """
        # ExtraHosts 未適用でも届くよう、解決できなければ gateway へフォールバック
        endpoint = self._resolve_endpoint()
        logger.info("MAVLink 接続先: %s", endpoint)
        vehicle = mavutil.mavlink_connection(endpoint)
        # autopilot(非GCS)の HEARTBEAT を最大30秒待って特定する。
        # udpout(client)は UDP なので単発 heartbeat だと Router が返送先を
        # 学習し損ねる。毎回送って確実に登録させる。
        hb = None
        got_any = False  # 何らかの MAVLink が届いたか(切り分け用)
        for _ in range(30):
            self._send_gcs_heartbeat(vehicle)
            msg = vehicle.recv_match(blocking=True, timeout=1)
            if msg is None:
                continue
            got_any = True
            if (msg.get_type() == "HEARTBEAT"
                    and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID):
                hb = msg
                break
        if hb is None:
            vehicle.close()
            raise TimeoutError(
                "オートパイロットの HEARTBEAT を受信できませんでした "
                f"(endpoint={endpoint}). {self._diagnose_endpoint(got_any, endpoint)}"
            )

        self.target_system = hb.get_srcSystem() or 1
        # component=0 は COMMAND_LONG で無視されるため既定の 1 を使う
        self.target_component = hb.get_srcComponent() or 1
        # 機体タイプから明示生成(mode_mapping() は直近 HEARTBEAT を見て
        # Plane/Copter を誤るため使わない)
        self.mode_map = mavutil.mode_mapping_byname(hb.type) or {}
        return vehicle

    @staticmethod
    def _default_gateway():
        """コンテナのデフォルトゲートウェイ(=Docker ホスト)IP を返す。

        /proc/net/route を読む。Linux コンテナ以外/取得不可なら None。
        """
        try:
            with open("/proc/net/route") as f:
                for line in f.readlines()[1:]:
                    fields = line.strip().split()
                    # Destination が 00000000(default) かつ RTF_GATEWAY(0x2)
                    if len(fields) >= 4 and fields[1] == "00000000" \
                            and int(fields[3], 16) & 0x2:
                        gw = int(fields[2], 16)
                        return socket.inet_ntoa(struct.pack("<L", gw))
        except Exception:  # noqa: BLE001
            return None
        return None

    def _resolve_endpoint(self) -> str:
        """接続文字列のホストが解決できなければ gateway へフォールバックする。

        BlueOS で ExtraHosts(host.docker.internal:host-gateway) が未適用でも、
        Docker のデフォルトゲートウェイ=ホストへ udpout できるようにする。
        """
        parts = CONNECTION_STRING.split(":")
        if len(parts) < 3:
            return CONNECTION_STRING
        proto, host, port = parts[0], parts[1], parts[2]
        try:
            socket.gethostbyname(host)
            return CONNECTION_STRING  # 解決できるのでそのまま
        except Exception:  # noqa: BLE001
            gw = self._default_gateway()
            if gw:
                logger.warning(
                    "'%s' を解決できないため gateway %s へフォールバックします",
                    host, gw,
                )
                return f"{proto}:{gw}:{port}"
            return CONNECTION_STRING

    @staticmethod
    def _diagnose_endpoint(got_any: bool, endpoint: str) -> str:
        """接続失敗時の原因切り分けメッセージを作る。

        - ホスト名を解決できない → ExtraHosts 未適用の疑い
        - 解決できるが無応答 → Router に該当エンドポイントが無い疑い
        - 何か届くが autopilot HB 無し → GCS のみが届いている疑い
        """
        host = None
        parts = endpoint.split(":")
        if len(parts) >= 3:
            host = parts[1]
        resolved = None
        if host:
            try:
                resolved = socket.gethostbyname(host)
            except Exception:  # noqa: BLE001
                resolved = None
        if resolved is None:
            return (
                f"ホスト '{host}' を名前解決できません。"
                "Dockerfile/インストール設定の ExtraHosts"
                "(host.docker.internal:host-gateway) を確認してください"
            )
        if got_any:
            return (
                f"'{host}'({resolved}) から MAVLink は届いていますが "
                "autopilot の HEARTBEAT がありません。宛先ポートに機体データが"
                "流れているか(GCS 用ポートに繋いでいないか)を確認してください"
            )
        return (
            f"'{host}'({resolved}) へ到達できますが応答がありません。"
            "BlueOS の MAVLink Endpoints に該当ポートの UDP Server があるか、"
            "MAV_ENDPOINT の宛先ポートを確認してください"
        )

    def _send_gcs_heartbeat(self, vehicle=None) -> None:
        """GCS として heartbeat を送る。

        udpout(client)接続では、これを送り続けないと Router が返送先を
        学習・維持できず、テレメトリが届かなくなる。
        """
        veh = vehicle or self.vehicle
        if veh is None:
            return
        try:
            veh.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0,
            )
        except Exception:  # noqa: BLE001
            pass

    def _request_data_streams(self) -> None:
        """位置系を含むデータストリームを要求する。"""
        try:
            self.vehicle.mav.request_data_stream_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                4,  # 4Hz
                1,  # start
            )
        except Exception:  # noqa: BLE001
            logger.exception("データストリーム要求に失敗しました")

    # ---- 受信ループ -------------------------------------------------------

    async def _recv_loop(self) -> None:
        """MAVLink を継続受信して状態を更新する。"""
        loop = asyncio.get_running_loop()
        last_hb = 0.0
        while self.state.connected:
            # Router が返送先を維持できるよう約1秒ごとに GCS heartbeat を送る
            now = time.monotonic()
            if now - last_hb >= 1.0:
                self._send_gcs_heartbeat()
                last_hb = now
            try:
                msg = await loop.run_in_executor(None, self._blocking_recv)
            except Exception:  # noqa: BLE001
                logger.exception("MAVLink 受信で例外が発生しました")
                await asyncio.sleep(0.1)
                continue
            if msg is None:
                continue
            self._handle_message(msg)

    def _blocking_recv(self):
        """タイムアウト付きブロッキング受信(executor 上で実行)。"""
        if self.vehicle is None:
            return None
        return self.vehicle.recv_match(blocking=True, timeout=0.1)

    def _handle_message(self, msg) -> None:
        mtype = msg.get_type()
        # 自機(確定した target sys/comp)以外は無視する。
        # Router 経由では GCS 等の HEARTBEAT が混ざり mode/armed が点滅するため。
        if mtype in ("GLOBAL_POSITION_INT", "HEARTBEAT"):
            if msg.get_srcSystem() != self.target_system:
                return
            if mtype == "HEARTBEAT" and msg.get_srcComponent() != self.target_component:
                return
        if mtype == "GLOBAL_POSITION_INT":
            self.state.latitude = msg.lat / 1e7
            self.state.longitude = msg.lon / 1e7
            # relative_alt(mm→m)。無ければ alt。
            if getattr(msg, "relative_alt", None) is not None:
                self.state.altitude = msg.relative_alt / 1000.0
            else:
                self.state.altitude = msg.alt / 1000.0
            hdg = msg.hdg
            if hdg != 65535:  # 65535 は不明値
                self.state.heading = int(hdg / 100)
        elif mtype == "HEARTBEAT":
            # ここに来るのは自機のオートパイロットのみ(上部でフィルタ済み)
            # ARM 判定は base_mode & SAFETY_ARMED で行う
            self.state.armed = bool(
                msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )
            self.state.mode = self._resolve_mode(msg.custom_mode)
        elif mtype == "COMMAND_ACK":
            # コマンドの受理/拒否理由を UI へ通知し、ACK 待ち合わせにも使う
            self._acks[msg.command] = msg.result
            cmd_name = mavutil.mavlink.enums["MAV_CMD"].get(msg.command)
            result_name = mavutil.mavlink.enums["MAV_RESULT"].get(msg.result)
            cmd_label = cmd_name.name if cmd_name else str(msg.command)
            result_label = result_name.name if result_name else str(msg.result)
            if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                self._emit(f"コマンド拒否: {cmd_label} -> {result_label}")
            else:
                self._emit(f"コマンド受理: {cmd_label}")
        elif mtype == "STATUSTEXT":
            # PreArm 等のメッセージをそのまま UI へ
            text = msg.text
            if isinstance(text, bytes):
                text = text.decode("utf-8", "replace")
            self._emit(f"機体: {text}")

    def _resolve_mode(self, custom_mode: int) -> str:
        """custom_mode をモード名へ逆引きする(明示モードマップを使用)。"""
        for name, num in self.mode_map.items():
            if num == custom_mode:
                return name
        return self.state.mode

    # ---- コマンド ---------------------------------------------------------

    def _ensure_connected(self) -> bool:
        return self.vehicle is not None and self.state.connected

    async def _ensure_guided(self) -> bool:
        """GUIDED への切替を試みる(最大5秒待機)。"""
        self.set_mode("GUIDED")
        loop = asyncio.get_running_loop()
        for _ in range(50):
            if self.state.mode == "GUIDED":
                return True
            await asyncio.sleep(0.1)
        return self.state.mode == "GUIDED"

    def set_mode(self, mode_name: str) -> None:
        """set_mode() でモード変更する(command_long は使わない)。"""
        if not self._ensure_connected():
            return
        try:
            mode_name = mode_name.upper()
            if mode_name not in self.mode_map:
                logger.warning("未知のモード: %s", mode_name)
                return
            self.vehicle.set_mode(self.mode_map[mode_name])
        except Exception:  # noqa: BLE001
            logger.exception("モード変更に失敗しました")

    def arm(self, value: int, force: bool = False) -> None:
        if not self._ensure_connected():
            return
        # force=True のとき param2=21196(強制アーム/ディスアームの魔法値)
        param2 = 21196.0 if force else 0.0
        try:
            self.vehicle.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                float(value), param2, 0, 0, 0, 0, 0,
            )
        except Exception:  # noqa: BLE001
            logger.exception("アーム/ディスアームに失敗しました")

    async def _wait_ack(self, command: int, timeout: float = 3.0) -> int | None:
        """指定コマンドの COMMAND_ACK を待つ。result 値、無応答なら None。"""
        self._acks.pop(command, None)
        steps = int(timeout / 0.1)
        for _ in range(steps):
            if command in self._acks:
                return self._acks.pop(command)
            await asyncio.sleep(0.1)
        return None

    def takeoff_send(self, altitude: float) -> None:
        try:
            self.vehicle.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0, 0, 0, 0, 0, 0, float(altitude),
            )
        except Exception:  # noqa: BLE001
            logger.exception("離陸コマンド送信に失敗しました")

    def land(self) -> None:
        if not self._ensure_connected():
            return
        try:
            self.vehicle.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0,
                0, 0, 0, 0, 0, 0, 0,
            )
        except Exception:  # noqa: BLE001
            logger.exception("着陸コマンド送信に失敗しました")

    def goto_send(self, lat: float, lon: float, alt: float) -> None:
        try:
            # 位置のみ有効(速度・加速度・yaw を無効化)
            type_mask = 0b0000111111111000
            self.vehicle.mav.set_position_target_global_int_send(
                0,  # time_boot_ms
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                type_mask,
                int(lat * 1e7),
                int(lon * 1e7),
                float(alt),
                0, 0, 0,  # vx, vy, vz
                0, 0, 0,  # afx, afy, afz
                0, 0,     # yaw, yaw_rate
            )
        except Exception:  # noqa: BLE001
            logger.exception("GoTo コマンド送信に失敗しました")

    # ---- コマンドディスパッチ --------------------------------------------

    async def handle_command(self, data: dict) -> str:
        cmd = data.get("type")

        if cmd == "connect":
            return await self.connect()

        if not self._ensure_connected():
            return "未接続です。先に接続してください"

        if cmd in ("arm", "disarm"):
            value = 1 if cmd == "arm" else 0
            force = bool(data.get("force", False))
            self.arm(value, force=force)
            target = f"sys={self.target_system} comp={self.target_component}"
            result = await self._wait_ack(
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
            )
            label = "アーム" if value == 1 else "ディスアーム"
            if result is None:
                return (
                    f"{label}: ACK が返りません({target})。"
                    "送信先 component 不一致の可能性があります"
                )
            if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                return f"{label}を受理しました"
            rname = mavutil.mavlink.enums["MAV_RESULT"].get(result)
            rlabel = rname.name if rname else str(result)
            return f"{label}が拒否されました: {rlabel}(理由は機体メッセージ参照)"
        if cmd == "takeoff":
            altitude = float(data.get("altitude", 0))
            if not await self._ensure_guided():
                return "GUIDED への切替に失敗しました"
            self.takeoff_send(altitude)
            return f"離陸コマンドを送信しました(高度 {altitude}m)"
        if cmd == "land":
            self.land()
            return "着陸コマンドを送信しました"
        if cmd == "goto":
            lat = float(data.get("latitude", 0))
            lon = float(data.get("longitude", 0))
            alt = float(data.get("altitude", 0))
            if not await self._ensure_guided():
                return "GUIDED への切替に失敗しました"
            self.goto_send(lat, lon, alt)
            return f"GoTo コマンドを送信しました({lat}, {lon}, {alt}m)"
        if cmd == "mode":
            mode_name = str(data.get("mode", "")).upper()
            self.set_mode(mode_name)
            return f"モード変更コマンドを送信しました({mode_name})"

        return f"未知のコマンド: {cmd}"


drone = DroneManager()


@app.get("/")
async def index():
    return FileResponse(str(FRONTEND_DIR / "index.html"))


@app.get("/register_service")
async def register_service():
    """BlueOS Extension として左メニューへ登録するためのメタ情報。

    WebSocket はプロキシ非対応のため avoid_iframes=True とし、
    直接 http://<IP>:<PORT>/ を新ウィンドウで開かせる。
    """
    return {
        "name": "Drone Web App",
        "description": "ブラウザからドローンを制御する Web アプリ(MAVLink)",
        "icon": "mdi-drone",
        "company": "",
        "version": "1.0.0",
        "webpage": "",
        "api": "/docs",
        "avoid_iframes": True,
    }


async def _safe_send(ws: WebSocket, payload: dict, lock: asyncio.Lock) -> bool:
    """接続中のみ送信する。切断済みなら False を返し例外を出さない。

    送信は push タスクとコマンド応答で並行しうるため lock で直列化する。
    """
    if ws.application_state != WebSocketState.CONNECTED:
        return False
    async with lock:
        if ws.application_state != WebSocketState.CONNECTED:
            return False
        try:
            await ws.send_json(payload)
            return True
        except (WebSocketDisconnect, RuntimeError):
            # クライアント切断後の送信は無視する
            return False


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    send_lock = asyncio.Lock()

    # 接続直後に現在状態を即時送信
    await _safe_send(ws, {"type": "state", "state": drone.state.to_dict()}, send_lock)

    push_task = asyncio.create_task(_push_state(ws, send_lock))
    try:
        while True:
            data = await ws.receive_json()
            message = await drone.handle_command(data)
            if not await _safe_send(
                ws, {"type": "status", "message": message}, send_lock
            ):
                break  # 切断済み
    except WebSocketDisconnect:
        logger.info("WebSocket 切断")
    except Exception:  # noqa: BLE001
        logger.exception("WebSocket でエラーが発生しました")
    finally:
        push_task.cancel()


async def _push_state(ws: WebSocket, lock: asyncio.Lock) -> None:
    """テレメトリー状態を定期的にクライアントへ送る。"""
    try:
        while True:
            # 溜まった機体側イベント(ACK / STATUSTEXT)を先に流す
            while not drone.events.empty():
                message = drone.events.get_nowait()
                if not await _safe_send(
                    ws, {"type": "status", "message": message}, lock
                ):
                    return  # 切断済み
            payload = {"type": "state", "state": drone.state.to_dict()}
            if not await _safe_send(ws, payload, lock):
                return  # 切断済み
            await asyncio.sleep(0.25)
    except asyncio.CancelledError:
        return
    except Exception:  # noqa: BLE001
        logger.exception("状態送信で例外が発生しました")


# 静的ファイルは最後にマウント(ルート等のルーティングを優先)
app.mount("/static", StaticFiles(directory=str(FRONTEND_DIR)), name="static")


if __name__ == "__main__":
    import uvicorn

    uvicorn.run("main:app", host="0.0.0.0", port=PORT, reload=True)
