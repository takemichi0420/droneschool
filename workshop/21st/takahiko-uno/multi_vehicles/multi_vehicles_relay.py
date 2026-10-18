#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
複数機体リレー運行スクリプト（ローバー → ボート → コプター）
※回送対応版（コプター → ボート → ローバー の順に配送先から出発地へ戻す対応）


ファイルの関係：
multi_vehicle_dialog_separated.bat   Windows側で3機のSITLを起動（機体ごとにフォルダを分ける）
        │
        │ TCP接続（5760/5770/5780、Mission Planner使用時は5762/5772/5782）
        ▼
multi_vehicles_relay.py   運行の本体。接続 → ミッション書き込み → アーム → 到着判定
        │ import
        ▼
routes.py   ルート座標・速度・高度の定義。ここからミッションを生成
        │
        └→ mission_*.waypoints   生成物（確認用）。運行では読み込まない

README.md   使い方と設計の説明（プログラムからは参照されない）


課題要件:
  ルート毎に決められた種類の機体を運行する。
    1. 滑川駅       → 対岸ポート    : ローバー  (TCP 5760)
    2. 対岸ポート   → メインポート  : ボート    (TCP 5770)
    3. メインポート → セブンイレブン: コプター  (TCP 5780)
  前の機体が到着し、荷物を載せ替えた後に次の機体が出発する。

ミッション:
  指定座標に基づくルートは routes.py に定義してあり、本スクリプトが MAVLink の
  ミッションを生成して各機体へ自動でアップロードする（ミッション作成もプログラム側で行う）。
  Mission Planner で手動作成したミッションを使う場合は --no-upload-missions を付ける
  （その場合は機体にミッションが書き込まれている前提となる）。

初期位置:
  運行を始める前に、各機体をそのルートの出発地へ配置する。
    ローバー: 滑川駅 (35.876991, 140.348026)
    ボート  : 対岸ポート (35.879768, 140.348495)
    コプター: メインポート (35.878275, 140.338069)
  既に出発地にいる場合は何もしない。前回の運行で目的地に残っている場合は、
  SITL の初期位置(SIM_OPOS_*)を設定して機体を再起動し、出発地に戻してから運行する。
  （--no-set-start-position を付けると、この配置処理を省略できる）

前提:
  - ローバー / ボートは「アーム → ミッション開始（AUTO）」
  - コプターは「アーム → 離陸 → ミッション開始（AUTO）」
  - 3機はそれぞれ別ポートで待ち受けている（SITL 3機同時起動、または MissionPlanner 経由）。

処理の流れ（1レグあたり）:
  1. 接続（wait_heartbeat）→ 出発地に配置（必要なら機体を再起動）
  2. ミッションをアップロード（--no-upload-missions 指定時は省略）し、
     ダウンロードして検証（ホーム以外に1つ以上コマンドがあるか）
     - 先頭の NAV_TAKEOFF から離陸高度を取得（コプター用）
     - 最終ウェイポイント座標を取得（到着判定のフォールバック用）
  3. 巡航速度を出すための機体パラメータを設定し、ミッション実行位置を先頭に戻す
     （再実行しても必ず最初から走る）
  4. アーム
       ローバー/ボート: HOLD でアーム → AUTO へ変更 → MISSION_START
       コプター:        GUIDED でアーム → 離陸 → AUTO へ変更 → MISSION_START
  5. 到着待ち（下記いずれかを検知したら到着とみなす）
       - MISSION_ITEM_REACHED が最終シーケンス番号に到達
       - MISSION_CURRENT の mission_state == MISSION_STATE_COMPLETE
       - STATUSTEXT に "mission complete" 等が出た
       - 最終ウェイポイントから ARRIVE_RADIUS_M 以内に ARRIVE_SETTLE_SEC 秒留まった
       - ディスアームされた（ミッション末尾の LAND / DISARM で自動停止した場合）
  6. 到着処理
       ローバー/ボート: HOLD へ変更してディスアーム
       コプター:        必要なら LAND して着陸・ディスアームを待つ
  7. 荷物の載せ替え待ち（--transfer-sec 秒のカウントダウン、--confirm 指定時は Enter 待ち）
     → 完了後に次のレグへ

接続先:
  指定した host/ポートで MAVLink が流れていない場合、応答する組み合わせを自動で探す。
  ホストは 指定値 → WSLから見たWindows側のIP、ポートは 指定値 → +2 → +3 → -2 の順で試す。
  SITLの1本目のポート(5760/5770/5780)は Mission Planner 等が先に使っていると
  TCPは繋がってもデータが来ないため、HEARTBEATの受信で判定している。

使い方:
  # 通常はこれだけ（接続先は自動判定）
  python3 multi_vehicles_relay.py

  # 接続先を明示する場合（IPは機体が動いているPCのもの）
  python3 multi_vehicles_relay.py --host 192.168.x.x --rover-port 5762

  # Mission Planner でSITLを起動した構成に合わせる（ポート +2 ・Windows側IPの自動判定）
  python3 multi_vehicles_relay.py --mission-planner

  # 載せ替えを自動の10秒待ちではなく、オペレーターの Enter 入力で進める
  python3 multi_vehicles_relay.py --confirm

  # 動作確認用: 1機だけ運行する
  python3 multi_vehicles_relay.py --legs copter

  # 生成したミッションを .waypoints で書き出す（Mission Planner で確認できる）
  python3 multi_vehicles_relay.py --export-waypoints .
"""

import argparse
import contextlib
import io
import math
import os
import sys
import time
import unicodedata

from pymavlink import mavutil

import routes

# ==== 接続設定 ====
# 各機体は同一ホストの別ポートで待ち受けている前提（SITLを3機起動した構成）。
DEFAULT_HOST = "127.0.0.1"       # 例: Windows 側に接続する場合は "192.168.xx.xx"
DEFAULT_ROVER_PORT = 5760
DEFAULT_BOAT_PORT = 5770
DEFAULT_COPTER_PORT = 5780

# 自分（地上局側）の system_id / component_id
SOURCE_SYSTEM = 1
SOURCE_COMPONENT = 90

# ==== タイムアウト・しきい値 ====
DEFAULT_CONNECT_TIMEOUT = 30.0   # TCP接続を待つ時間[秒]（機体の起動待ちを含む）
# 接続先の自動判定で試すポートのずれ。0=指定どおり、2/3=SITLの2本目/3本目、-2=+2指定時の1本目
PORT_OFFSET_CANDIDATES = (0, 2, 3, -2)
HEARTBEAT_TIMEOUT = 30.0     # 接続時のHEARTBEAT待ち[秒]
ARM_TIMEOUT = 60.0           # アーム完了までの待ち[秒]（プリアームチェック通過待ちを含む）
ARM_RETRY_INTERVAL = 3.0     # アームコマンドの再送間隔[秒]
ACRO_BALANCE_DEFAULT = 1.0   # ACRO_BAL_ROLL/PITCH の既定値（不整合の修復に使う）
MODE_TIMEOUT = 30.0          # モード変更の確認待ち[秒]（この間コマンドを再送する）
MODE_RETRY_INTERVAL = 2.0    # モード変更コマンドの再送間隔[秒]
TAKEOFF_TIMEOUT = 60.0       # 離陸（目標高度到達）待ち[秒]
DEFAULT_LEG_TIMEOUT = 900.0  # 1レグ（出発〜到着）の最大待ち[秒]
DEFAULT_TAKEOFF_ALT = 10.0   # ミッションに NAV_TAKEOFF が無い場合の離陸高度[m]
ARRIVE_RADIUS_M = 5.0        # 最終ウェイポイントへの到達とみなす水平距離[m]
ARRIVE_SETTLE_SEC = 3.0      # 上記距離以内に留まり続ける必要がある時間[秒]
DEPARTURE_DISTANCE_M = 15.0  # 出発地点からこの距離[m]以上離れたら「出発した」とみなす
DEFAULT_TRANSFER_SEC = 10.0  # 荷物載せ替えの所要時間[秒]

# ==== 初期位置の設定（各機体をルートの出発地に配置する） ====
DEFAULT_START_TOLERANCE = 25.0   # 出発地に居るとみなす距離[m]
# 機体は到着後の停止までに惰性で流れる（ボートは水上で15m程度流れる実測値）。
# 港・駅の広さも考えると、この程度の余裕を見て「出発地にいる」と判定する。
START_POSITION_ALT = 10.0       # SITLの初期位置の高度[m]（地面の標高相当）
REBOOT_WAIT = 12.0              # 再起動後、再接続を試みるまでの待ち[秒]
REBOOT_RECONNECT_TIMEOUT = 90.0  # 再起動後に再接続できるまでの待ち[秒]
REBOOT_HEARTBEAT_TIMEOUT = 10.0  # 再接続1回あたりのHEARTBEAT待ち[秒]
REBOOT_POSITION_TIMEOUT = 180.0  # 再起動後、位置が取れるまで接続を張り直しながら待つ上限[秒]
POSITION_TIMEOUT = 60.0         # 位置情報を取得できるまでの待ち[秒]（GPS固定待ちを含む）
LINK_SILENT_SEC = 6.0           # この秒数メッセージが来なければ接続が切れたと判断する

# MISSION_CURRENT.mission_state の「ミッション完了」を表す値
MISSION_STATE_COMPLETE = mavutil.mavlink.MISSION_STATE_COMPLETE

# 位置情報（緯度経度）を持つミッションコマンド。最終ウェイポイント座標の抽出に使う。
POSITIONAL_NAV_CMDS = (
    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
    mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
    mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
    mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT,
)

# ミッション完了を示す STATUSTEXT（機種・FWバージョンで文言が異なるため複数用意）
MISSION_DONE_TEXTS = ("mission complete", "mission finished", "reached destination")


class Leg:
    """1区間（レグ）の運行定義。ルート（座標）は routes.py 側に持つ。"""

    def __init__(self, name, port, kind, needs_takeoff, arm_mode):
        self.name = name                    # 表示名（ローバー等）
        self.port = port                    # 接続先TCPポート
        self.kind = kind                    # "rover" / "boat" / "copter"
        self.needs_takeoff = needs_takeoff  # 離陸コマンドを送るか（コプターのみ True）
        self.arm_mode = arm_mode            # アームする際のモード
        self.route_def = routes.ROUTES[kind]  # ルート定義（座標・巡航速度・高度）

    @property
    def route(self):
        """表示用のルート名（例: 滑川駅 → 対岸ポート）。"""
        return "%s → %s" % (self.route_def.origin_name, self.route_def.destination_name)


def build_legs(args):
    """運行するレグ（実行順）を組み立てる。

    --legs で一部だけを指定した場合は、その機体だけを運行する（動作確認用）。
    --return では、配送先から出発地へ戻す回送として、順序とルートを逆にする。
    """
    all_legs = [
        Leg("ローバー", args.rover_port, kind="rover", needs_takeoff=False, arm_mode="HOLD"),
        Leg("ボート", args.boat_port, kind="boat", needs_takeoff=False, arm_mode="HOLD"),
        Leg("コプター", args.copter_port, kind="copter", needs_takeoff=True, arm_mode="GUIDED"),
    ]
    if args.legs:
        selected = [name.strip().lower() for name in args.legs.split(",") if name.strip()]
        known = [leg.kind for leg in all_legs]
        for name in selected:
            if name not in known:
                raise SystemExit(
                    "--legs に不正な機体種別が指定されました: '%s'（指定可能: %s）"
                    % (name, ", ".join(known)))
        # 指定順ではなく、課題のルート順を維持する
        all_legs = [leg for leg in all_legs if leg.kind in selected]

    if args.return_flight:
        # 回送: コプター → ボート → ローバー の順に、各ルートを逆向きに運行する
        all_legs.reverse()
        for leg in all_legs:
            leg.route_def = routes.reversed_route(leg.route_def)

    return all_legs


# ---------------------------------------------------------------------------
# 表示ヘルパー
# ---------------------------------------------------------------------------

def print_banner(text):
    bar = "=" * 70
    print("\n" + bar)
    print(text)
    print(bar)


def print_step(text):
    print("  - %s" % text)


def pad(text, width):
    """全角文字を2文字幅として数え、表示幅を width に揃える（結果一覧の桁合わせ用）。"""
    display_width = sum(2 if unicodedata.east_asian_width(ch) in "WFA" else 1 for ch in text)
    return text + " " * max(0, width - display_width)


# ---------------------------------------------------------------------------
# 接続・共通ユーティリティ
# ---------------------------------------------------------------------------

def detect_gateway_host():
    """WSL から見た Windows 側のIP（デフォルトゲートウェイ）を返す。取得できなければ None。

    Mission Planner を Windows 側で動かし、本スクリプトを WSL で動かす構成で使う。
    Linux 以外（Windows 上で直接実行）では /proc/net/route が無いので None を返す。
    """
    try:
        with open("/proc/net/route", "r") as route_table:
            for line in route_table.readlines()[1:]:
                fields = line.split()
                # Destination が 00000000 の行がデフォルトルート。Gateway はリトルエンディアンのhex
                if len(fields) > 2 and fields[1] == "00000000":
                    gateway = int(fields[2], 16)
                    return "%d.%d.%d.%d" % (gateway & 0xFF, (gateway >> 8) & 0xFF,
                                            (gateway >> 16) & 0xFF, (gateway >> 24) & 0xFF)
    except (OSError, ValueError):
        pass
    return None


def resolve_direction(args):
    """運行方向（配送 / 回送 / 自動判定）を決める。

    既定は "auto"（各機体が配送の出発地にいるかどうかで決める）。
    --delivery / --return を付けた場合はその方向に固定する。
    """
    if args.return_flight and args.force_delivery:
        raise SystemExit("--return と --delivery は同時に指定できません。")
    if args.return_flight:
        args.direction = "return"
    elif args.force_delivery:
        args.direction = "delivery"
    else:
        args.direction = "auto"


def resolve_connection_settings(args):
    """未指定のホスト・ポートを決める。

    --mission-planner 指定時は Mission Planner 構成に合わせる。
    Mission Planner でSITLを起動すると 5760/5770/5780 は SITL 本体と Mission Planner が
    使っているため、追加クライアント用の +2 のポート（5762/5772/5782）に接続する。
    ホストは、WSL から Windows 側へ繋ぐ場合のデフォルトゲートウェイを使う。
    どちらも明示指定があればそちらを優先する。
    """
    port_offset = 2 if args.mission_planner else 0

    if args.host is None:
        args.host = DEFAULT_HOST
        if args.mission_planner:
            args.host = detect_gateway_host() or DEFAULT_HOST

    for name, default in (("rover_port", DEFAULT_ROVER_PORT),
                          ("boat_port", DEFAULT_BOAT_PORT),
                          ("copter_port", DEFAULT_COPTER_PORT)):
        if getattr(args, name) is None:
            setattr(args, name, default + port_offset)


def mavlink_is_flowing(host, port, timeout=4.0):
    """host:port に MAVLink が流れているか（HEARTBEATが来るか）を確かめる。

    TCPが繋がるだけでは不十分。Mission Planner 等が先にそのポートを使っていると
    接続はできてもデータが来ないため、HEARTBEAT の受信で判定する。
    """
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            probe = mavutil.mavlink_connection(
                "tcp:%s:%d" % (host, port), source_system=SOURCE_SYSTEM,
                source_component=SOURCE_COMPONENT, retries=0)
            heartbeat = probe.wait_heartbeat(timeout=timeout)
            probe.close()
    except OSError:
        return False
    return heartbeat is not None


def autodetect_connection(args, legs):
    """3機とも MAVLink が流れている host / ポートの組み合わせを探して args を補正する。

    SITL は 1本目のポート（5760/5770/5780）を Mission Planner 等に使われていることが多く、
    その場合は2本目・3本目（+2 / +3 = 5762/5763 など）に接続する必要がある。
    また WSL で実行する場合、Windows 側のSITLへは 127.0.0.1 ではなく
    Windows のIPを指定する必要がある。この組み合わせを総当りで確かめる。

    指定された設定を最初に試し、そのまま使えるなら何も変更しない。
    """
    hosts = [args.host]
    gateway = detect_gateway_host()
    if gateway is not None and gateway not in hosts:
        hosts.append(gateway)

    base_ports = [leg.port for leg in legs]
    for host in hosts:
        for offset in PORT_OFFSET_CANDIDATES:
            ports = [port + offset for port in base_ports]
            if any(port < 1 for port in ports):
                continue
            if all(mavlink_is_flowing(host, port) for port in ports):
                if host == args.host and offset == 0:
                    return      # 指定どおりで問題なし
                print_step("接続先を自動判定しました: %s ポート %s"
                           % (host, " / ".join(str(port) for port in ports)))
                print_step("（指定は %s ポート %s でしたが、MAVLink が流れていませんでした）"
                           % (args.host, " / ".join(str(port) for port in base_ports)))
                args.host = host
                for leg, port in zip(legs, ports):
                    leg.port = port
                args.rover_port, args.boat_port, args.copter_port = (
                    DEFAULT_ROVER_PORT + offset, DEFAULT_BOAT_PORT + offset,
                    DEFAULT_COPTER_PORT + offset)
                return

    print_step("[警告] 応答する接続先を見つけられませんでした。指定の設定で接続を試みます。")


def open_connection(connection_string, connect_timeout):
    """TCP接続を確立する。まだ機体が起動していない場合は起動を待つ。

    機体（SITL）が立ち上がる前に実行すると接続を拒否される（Connection refused）ため、
    connect_timeout 秒までは待ち直す。時間内に繋がらない場合は原因の候補を添えて返す。
    """
    deadline = time.time() + connect_timeout
    notified = False
    while True:
        try:
            # retries=0: pymavlink 側のリトライ表示に任せず、ここで待ち方を制御する
            return mavutil.mavlink_connection(
                connection_string, source_system=SOURCE_SYSTEM,
                source_component=SOURCE_COMPONENT, retries=0)
        except OSError as e:
            if time.time() >= deadline:
                raise ConnectionError(
                    "%s に接続できませんでした（%s）。次を確認してください。\n"
                    "    - その機体（SITL）が起動していますか（このポートで待ち受けているか）\n"
                    "      例: sim_vehicle.py -v Rover -I0 --no-mavproxy "
                    "--custom-location=35.876991,140.348026,10,0\n"
                    "    - MAVProxy / Mission Planner が同じポートを使っていませんか\n"
                    "      （MAVProxy 経由なら 5762 等になります。--rover-port 等で指定してください）\n"
                    "    - 別PC上の機体に接続する場合、--host にそのPCのIPを指定していますか"
                    % (connection_string, e))
            if not notified:
                print_step("まだ接続できません（%s）。機体の起動を待ちます（最大 %.0f 秒）..."
                           % (e, connect_timeout))
                notified = True
            time.sleep(2.0)


def reconnect_after_reboot(host, port, leg, connect_timeout):
    """再起動した機体へ接続し直す。

    再起動直後は、TCP接続そのものはできても機体がまだ通信を始めておらず、
    掴んだ接続がすぐ切れる（EOF）ことがある。その場合は接続を捨てて張り直す。
    pymavlink は EOF のたびに標準出力へメッセージを出し続けるため、
    再接続を試している間だけ標準出力を捨てる。
    """
    connection_string = "tcp:%s:%d" % (host, port)
    deadline = time.time() + REBOOT_RECONNECT_TIMEOUT
    attempt = 0
    while True:
        attempt += 1
        master = open_connection(connection_string, connect_timeout)
        with contextlib.redirect_stdout(io.StringIO()):
            # 掴んだ接続がすぐ切れることがあるため、HEARTBEATを2回受けて安定を確認する
            heartbeat = master.wait_heartbeat(timeout=REBOOT_HEARTBEAT_TIMEOUT)
            stable = (heartbeat is not None
                      and master.target_system != 0
                      and master.wait_heartbeat(timeout=5) is not None)
        if stable:
            print_step("再接続しました（%d回目） target_system=%d"
                       % (attempt, master.target_system))
            return master

        master.close()
        if time.time() >= deadline:
            raise TimeoutError(
                "%s の再起動後、%.0f秒以内に再接続できませんでした。"
                % (leg.name, REBOOT_RECONNECT_TIMEOUT))
        print_step("機体の起動を待っています（%d回目の再接続を試行中）..." % attempt)
        time.sleep(3.0)


def find_talking_port(host, port, offsets=(2, -2)):
    """近いポート番号に MAVLink が流れていないか確かめ、見つかればその番号を返す。

    Mission Planner や MAVProxy が SITL のポート（5760等）を先に掴んでいると、
    こちらは「TCPは繋がるがMAVLinkが流れてこない」状態になる。その場合、
    追加クライアント用のポート（5762等）で待ち受けているので、それを案内するために探す。
    """
    for offset in offsets:
        candidate = port + offset
        if candidate < 1:
            continue
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                probe = mavutil.mavlink_connection(
                    "tcp:%s:%d" % (host, candidate), source_system=SOURCE_SYSTEM,
                    source_component=SOURCE_COMPONENT, retries=0)
                heartbeat = probe.wait_heartbeat(timeout=5)
                probe.close()
        except OSError:
            continue
        if heartbeat is not None:
            return candidate
    return None


def connect_vehicle(host, port, connect_timeout=None):
    """機体へ接続し、HEARTBEAT を受信してから接続オブジェクトを返す。"""
    connection_string = "tcp:%s:%d" % (host, port)
    print_step("接続します: %s" % connection_string)
    master = open_connection(
        connection_string,
        DEFAULT_CONNECT_TIMEOUT if connect_timeout is None else connect_timeout)

    if master.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT) is None:
        master.close()
        message = (
            "%s は接続できましたが、HEARTBEAT が %.0f秒以内に届きませんでした。\n"
            "    Mission Planner や MAVProxy がこのポートを先に使っていると、"
            "TCPは繋がってもMAVLinkが流れてきません。"
            % (connection_string, HEARTBEAT_TIMEOUT))
        alternative = find_talking_port(host, port)
        if alternative is not None:
            message += ("\n    tcp:%s:%d では MAVLink が流れています。"
                        "こちらを指定してください（--mission-planner でも同じ設定になります）。"
                        % (host, alternative))
        else:
            message += "\n    機体（SITL）が起動しているか、ホスト/ポートを確認してください。"
        raise TimeoutError(message)

    print_step("接続完了 target_system=%d, target_component=%d"
               % (master.target_system, master.target_component))
    return master


def send_heartbeat(master):
    """地上局側の HEARTBEAT を送信する。

    長時間の待機ループ中に無通信になると、機体側の GCS フェイルセーフを
    誘発する可能性があるため、待機中も定期的に送信する。
    """
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
        0, 0, 0)


def request_message_interval(master, message_id, hz):
    """指定メッセージの送信レートを要求する（到着判定用の位置情報等）。"""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0, message_id, int(1e6 / hz), 0, 0, 0, 0, 0)


def get_distance_metres(lat1, lon1, lat2, lon2):
    """2点間の水平距離[m]を近似計算する（数km規模までは十分な精度）。"""
    dlat = lat2 - lat1
    dlon = (lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2.0))
    return math.sqrt(dlat * dlat + dlon * dlon) * 1.113195e5


def initial_bearing(origin, target):
    """origin から target へ向かう方位[度]（0=北）。初期の機首方位に使う。"""
    lat1, lon1 = math.radians(origin[0]), math.radians(origin[1])
    lat2, lon2 = math.radians(target[0]), math.radians(target[1])
    dlon = lon2 - lon1
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def get_position(master, timeout=POSITION_TIMEOUT):
    """機体の現在位置(lat, lon)を取得する。取得できなければ None。

    接続が切れている場合（再起動直後に掴んだ接続がすぐ落ちる等）は、
    タイムアウトいっぱい粘らず、無通信が LINK_SILENT_SEC 続いた時点で諦める。
    呼び出し側が接続を張り直せるようにするため。
    """
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 2)
    deadline = time.time() + timeout
    last_received = time.time()
    while time.time() < deadline:
        msg = master.recv_match(type=["GLOBAL_POSITION_INT", "HEARTBEAT"],
                                blocking=True, timeout=2)
        if msg is not None:
            last_received = time.time()
            if msg.get_type() == "GLOBAL_POSITION_INT" and msg.lat != 0:
                return (msg.lat / 1e7, msg.lon / 1e7)
        elif time.time() - last_received > LINK_SILENT_SEC:
            return None     # 接続が切れている
        send_heartbeat(master)
    return None


def get_param(master, name, timeout=5.0):
    """パラメータを1つ読む。存在しない/応答が無い場合は None。

    機体に無いパラメータ名の場合、対応FWは PARAM_ERROR を返すので待たずに打ち切る
    （バージョン違いでパラメータ名を順に試す場面が速くなる）。
    """
    master.mav.param_request_read_send(
        master.target_system, master.target_component, name.encode(), -1)
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type=["PARAM_VALUE", "PARAM_ERROR"], blocking=True, timeout=2)
        if msg is None:
            continue
        if msg.param_id.strip("\x00") != name:
            continue
        if msg.get_type() == "PARAM_ERROR":
            return None
        return msg.param_value
    return None


def set_param(master, name, value, timeout=5.0):
    """パラメータを設定し、読み戻した値を返す（確認できなければ None）。"""
    master.mav.param_set_send(
        master.target_system, master.target_component, name.encode(),
        float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
        if msg is not None and msg.param_id.strip("\x00") == name:
            return msg.param_value
    return None


def set_mode(master, mode):
    """フライトモードを変更し、HEARTBEAT で反映を確認する。

    起動直後の機体は、位置推定(EKF)等の準備が終わるまでモード変更を拒否することがある。
    1回送って諦めると「AUTOに入れない」で止まってしまうため、タイムアウトまで
    コマンドを再送しながら待つ。拒否された理由（COMMAND_ACK / STATUSTEXT）も表示する。
    """
    mode_mapping = master.mode_mapping()
    if mode_mapping is None or mode not in mode_mapping:
        raise RuntimeError(
            "モード '%s' はこの機体で使用できません。使用可能: %s"
            % (mode, list(mode_mapping.keys()) if mode_mapping else "取得失敗"))

    mode_id = mode_mapping[mode]
    deadline = time.time() + MODE_TIMEOUT
    last_sent = 0.0
    last_result = None
    reported = set()

    while time.time() < deadline:
        now = time.time()
        if now - last_sent >= MODE_RETRY_INTERVAL:
            master.mav.command_long_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
            last_sent = now

        msg = master.recv_match(type=["HEARTBEAT", "COMMAND_ACK", "STATUSTEXT"],
                                blocking=True, timeout=1)
        if msg is None:
            continue
        msg_type = msg.get_type()

        if msg_type == "COMMAND_ACK":
            if msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                last_result = mavutil.mavlink.enums["MAV_RESULT"][msg.result].name
            continue
        if msg_type == "STATUSTEXT":
            text = msg.text.strip()
            # モード変更失敗の理由（"Mode change failed: ..." 等）は一度だけ表示する
            if text and text not in reported:
                reported.add(text)
                print_step("[機体メッセージ] %s" % text)
            continue

        if master.flightmode == mode:
            print_step("モード変更完了: %s" % mode)
            return

    raise TimeoutError(
        "モードを %s に変更できませんでした（現在モード=%s, %.0f秒間 再送, 応答=%s）。\n"
        "    起動直後は位置推定(EKF)やGPSの準備が終わるまで拒否されます。"
        "機体の状態を確認してください。"
        % (mode, master.flightmode, MODE_TIMEOUT, last_result))


def arm_vehicle(master):
    """アームする。プリアームチェック待ちのため、コマンドを再送しながら待つ。

    アームできない理由（PreArm 等）は STATUSTEXT で流れてくるので、そのまま表示する。
    """
    print_step("アームします。")
    deadline = time.time() + ARM_TIMEOUT
    last_sent = 0.0
    reported = set()    # 表示済みの機体メッセージ
    repaired = set()    # 修復を試みた機体メッセージ

    while time.time() < deadline:
        now = time.time()
        if now - last_sent >= ARM_RETRY_INTERVAL:
            master.arducopter_arm()   # COMPONENT_ARM_DISARM(param1=1) の送信。機種共通で使える
            send_heartbeat(master)
            last_sent = now

        msg = master.recv_match(type=["HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            text = msg.text.strip()
            if text and text not in reported:
                reported.add(text)
                print_step("[機体メッセージ] %s" % text)
                hint = prearm_hint(text)
                if hint:
                    print_step(hint)
                # 既知のパラメータ不整合が原因なら直して、アームの再送を続ける
                if text not in repaired:
                    repaired.add(text)
                    repair_prearm_params(master, text)
            continue
        if master.motors_armed():
            print_step("アーム完了")
            return

    raise TimeoutError(
        "アームが %.0f秒以内に完了しませんでした。"
        "上の [機体メッセージ] にプリアームチェックの失敗理由が出ています。" % ARM_TIMEOUT)


def prearm_hint(text):
    """機体のプリアーム失敗メッセージに対して、原因の見当を返す。"""
    if "ACRO_BAL_ROLL" in text or "ACRO_BAL_PITCH" in text:
        return ("→ ACRO_BAL_ROLL/PITCH が ATC_ANG_RLL_P/ATC_ANG_PIT_P より大きいと拒否されます。\n"
                "      複数機体のSITLを同じフォルダから起動すると eeprom.bin を共有するため、"
                "次回起動時にパラメータが壊れてこの症状が出ます。")
    return None


def repair_prearm_params(master, text):
    """アームを妨げている既知のパラメータ不整合を修正する。修正したら True。

    ACRO_BAL_ROLL / ACRO_BAL_PITCH は ATC_ANG_RLL_P / ATC_ANG_PIT_P 以下でなければ
    アームできない。ACROモード専用のパラメータで、この運行（GUIDED/AUTO/LAND）の
    飛行には影響しないため、上限内（既定値 1.0）に戻して運行を続けられるようにする。

    複数機体のSITLを同じフォルダから起動して eeprom.bin を共有している環境では、
    機体間でパラメータが混ざってこの症状が出るため、その場で直せるようにしている。
    """
    if "ACRO_BAL_ROLL" not in text and "ACRO_BAL_PITCH" not in text:
        return False

    repaired = False
    for balance_name, gain_name in (("ACRO_BAL_ROLL", "ATC_ANG_RLL_P"),
                                    ("ACRO_BAL_PITCH", "ATC_ANG_PIT_P")):
        balance = get_param(master, balance_name)
        gain = get_param(master, gain_name)
        if balance is None or gain is None or balance <= gain:
            continue
        target = min(ACRO_BALANCE_DEFAULT, gain)
        if set_param(master, balance_name, target) is None:
            print_step("[警告] %s を %.1f に修正できませんでした。" % (balance_name, target))
            continue
        print_step("%s が %s(%.1f) を超えていたため %.3f → %.1f に修正しました。"
                   % (balance_name, gain_name, gain, balance, target))
        repaired = True
    return repaired


def disarm_vehicle(master, timeout=30.0):
    """ディスアームする。"""
    print_step("ディスアームします。")
    deadline = time.time() + timeout
    last_sent = 0.0
    while time.time() < deadline:
        now = time.time()
        if now - last_sent >= 2.0:
            master.arducopter_disarm()
            send_heartbeat(master)
            last_sent = now
        if master.recv_match(type="HEARTBEAT", blocking=True, timeout=1) is None:
            continue
        if not master.motors_armed():
            print_step("ディスアーム完了")
            return True

    print_step("[警告] ディスアームを %.0f秒以内に確認できませんでした。" % timeout)
    return False


def wait_disarmed(master, timeout):
    """機体がディスアームされるまで待つ（着陸完了の確認に使う）。"""
    deadline = time.time() + timeout
    last_beat = 0.0
    while time.time() < deadline:
        now = time.time()
        if now - last_beat >= 1.0:
            send_heartbeat(master)
            last_beat = now
        if master.recv_match(type="HEARTBEAT", blocking=True, timeout=1) is None:
            continue
        if not master.motors_armed():
            return True
    return False


# ---------------------------------------------------------------------------
# ミッション
# ---------------------------------------------------------------------------

def decide_direction(args, outbound_legs):
    """各機体の現在位置から、配送ミッションか回送ミッションかを決める。

    判定は「その機体が配送の出発地にいるか、いないか」だけで行う。
      3機とも出発地にいる      → 配送（滑川駅 → … → セブンイレブン）
      出発地にいない機体がある → 回送（配送先にいるとみなして出発地へ戻す）
    位置が取得できない機体は「出発地にいない」として扱う。
    """
    print_banner("各機体の位置から運行方向を判定します")
    at_start = []
    for leg in outbound_legs:
        master = connect_vehicle(args.host, leg.port, args.connect_timeout)
        try:
            position = get_position(master)
        finally:
            master.close()

        start = leg.route_def.start
        if position is None:
            print_step("%s: 位置を取得できませんでした（出発地にいない扱い）。" % leg.name)
            at_start.append(False)
            continue

        offset = get_distance_metres(position[0], position[1], start[0], start[1])
        inside = offset <= args.start_tolerance
        print_step("%s: 出発地(%s)から %.1f m → %s"
                   % (leg.name, leg.route_def.origin_name, offset,
                      "出発地にいる" if inside else "出発地にいない"))
        at_start.append(inside)

    if all(at_start):
        print_step("3機とも出発地にいます → 配送ミッションを実行します。")
        return False
    if any(at_start):
        print_step("一部の機体が出発地にいません → 回送ミッション（機体を戻す）を実行します。")
    else:
        print_step("3機とも出発地にいません → 回送ミッション（機体を戻す）を実行します。")
    return True


def ensure_start_position(master, leg, args):
    """機体をそのルートの出発地に配置する（初期状態の設定）。接続オブジェクトを返す。

    ローバー=滑川駅 / ボート=対岸ポート / コプター=メインポート に置いてから運行を始める。
    既に出発地にいる場合は何もしない。離れている場合は、出発地から起動し直す手順を示して中止する。

    --reposition-by-reboot を付けた場合は、プログラム側で出発地に戻す:
      1. SIM_OPOS_LAT/LNG/ALT/HDG（SITLの初期位置）に出発地を設定
      2. 機体を再起動（PREFLIGHT_REBOOT_SHUTDOWN）
      3. 再接続して、出発地に配置されたことを位置情報で確認
    SITL は起動時に指定されたホーム位置（sim_vehicle の --custom-location や
    Mission Planner のシミュレーション開始位置）があればそちらに戻るため、
    再起動後も出発地に来ない場合は、その位置でSITLを起動し直す必要がある旨を伝えて中止する。
    なお Mission Planner が起動したSITLでは、再起動すると Mission Planner が用意していた
    追加クライアント用ポート（5762等）が失われ、機体に接続できなくなるため既定では行わない。

    実機（SIM_OPOS_LAT が無い機体）では再起動を行わず、警告だけ出してスキップする。
    """
    start = leg.route_def.start
    tolerance = args.start_tolerance

    position = get_position(master)
    if position is not None:
        offset = get_distance_metres(position[0], position[1], start[0], start[1])
        if offset <= tolerance:
            print_step("出発地(%s)に配置済みです（誤差 %.1f m）。"
                       % (leg.route_def.origin_name, offset))
            return master
        print_step("出発地(%s)から %.0f m 離れています。"
                   % (leg.route_def.origin_name, offset))
    else:
        print_step("現在位置を取得できませんでした。")

    if not args.reposition_by_reboot:
        raise RuntimeError(
            "%s が出発地(%s)にいません。\n"
            "    機体を出発地から起動し直してください。\n"
            "      - Mission Planner: その機体のシミュレーションを次の座標から開始する\n"
            "          %.6f, %.6f\n"
            "      - sim_vehicle.py: --custom-location=%.6f,%.6f,%.0f,%.0f を付けて起動する\n"
            "    プログラム側で戻す場合は --reposition-by-reboot を付けてください\n"
            "    （SITLを再起動して初期位置に戻します。Mission Planner が起動したSITLでは\n"
            "      再起動でMission Planner用の追加ポートが失われるため、通常は非推奨）。\n"
            "    配置の確認自体を省略する場合は --no-set-start-position を付けてください。"
            % (leg.name, leg.route_def.origin_name, start[0], start[1],
               start[0], start[1], START_POSITION_ALT,
               initial_bearing(start, leg.route_def.waypoints[1])))

    if master.motors_armed():
        raise RuntimeError(
            "%s がアーム済みのため初期位置を設定できません。ディスアームしてください。" % leg.name)

    # SITL かどうかの判定。実機を再起動させないための安全確認。
    if get_param(master, "SIM_OPOS_LAT") is None:
        print_step("[警告] SITL ではないため（SIM_OPOS_LAT が無い）初期位置の設定をスキップします。"
                   "機体を手動で出発地へ移動してください。")
        return master

    heading = initial_bearing(start, leg.route_def.waypoints[1])
    print_step("SITLの初期位置を設定します: %.6f, %.6f （機首方位 %.0f度）"
               % (start[0], start[1], heading))
    for name, value in (("SIM_OPOS_LAT", start[0]), ("SIM_OPOS_LNG", start[1]),
                        ("SIM_OPOS_ALT", START_POSITION_ALT), ("SIM_OPOS_HDG", heading)):
        if set_param(master, name, value) is None:
            print_step("[警告] %s の設定を確認できませんでした。" % name)

    print_step("機体を再起動します。")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0, 1, 0, 0, 0, 0, 0, 0)
    master.close()

    time.sleep(REBOOT_WAIT)

    # 再起動直後は、接続できても機体側の準備が終わっておらず接続が落ちることがある。
    # 位置が取れるまで、接続を張り直しながら待つ。
    deadline = time.time() + REBOOT_POSITION_TIMEOUT
    position = None
    master = None
    while time.time() < deadline:
        master = reconnect_after_reboot(args.host, leg.port, leg, args.connect_timeout)
        with contextlib.redirect_stdout(io.StringIO()):
            # 接続が落ちている間 pymavlink が大量に出力するので、その分だけ捨てる
            position = get_position(master, timeout=min(30.0, max(5.0, deadline - time.time())))
        if position is not None:
            break
        print_step("位置情報が取得できませんでした（接続が不安定）。接続を張り直します。")
        master.close()
        master = None
        time.sleep(3.0)

    if position is None:
        if master is not None:
            master.close()
        raise TimeoutError(
            "%s の再起動後、%.0f秒以内に位置情報を取得できませんでした。"
            "機体（SITL）が正常に起動し直したか確認してください。"
            % (leg.name, REBOOT_POSITION_TIMEOUT))
    offset = get_distance_metres(position[0], position[1], start[0], start[1])
    if offset <= tolerance:
        print_step("出発地(%s)に配置しました（誤差 %.1f m）。"
                   % (leg.route_def.origin_name, offset))
        return master

    raise RuntimeError(
        "%s を出発地に配置できませんでした（現在 %.6f, %.6f / 出発地 %.6f, %.6f / %.0f m のずれ）。\n"
        "    SITL は起動時に指定されたホーム位置に戻るため、その位置が出発地と異なります。\n"
        "    次のいずれかで、出発地からSITLを起動し直してください。\n"
        "      - sim_vehicle.py ... --custom-location=%.6f,%.6f,%.0f,%.0f\n"
        "      - Mission Planner のシミュレーション開始位置をこの座標にする\n"
        "    （--no-set-start-position を付けると、この確認と配置を省略できます）"
        % (leg.name, position[0], position[1], start[0], start[1], offset,
           start[0], start[1], START_POSITION_ALT, heading))


def set_first_existing_param(master, candidates):
    """(パラメータ名, 値, 単位) の候補を順に試し、機体に在る最初のものを設定する。

    ArduPilot はバージョンによってパラメータ名・単位が変わる
    （例: コプターの巡航速度は WPNAV_SPEED[cm/s] → WPNAV_SPD[m/s]）ため、
    存在するものを使う。
    """
    for name, value, unit in candidates:
        if get_param(master, name) is None:
            continue
        result = set_param(master, name, value)
        if result is None:
            print_step("[警告] %s を %.1f %s に設定できませんでした（速度が出ない可能性）。"
                       % (name, value, unit))
            return False
        print_step("%s = %.1f %s に設定しました。" % (name, result, unit))
        return True

    print_step("[警告] 速度のパラメータ（%s）が機体に見つかりませんでした（速度が出ない可能性）。"
               % " / ".join(c[0] for c in candidates))
    return False


def apply_sim_speedup(master, speedup):
    """SITL のシミュレーション速度を設定する（SIM_SPEEDUP）。

    10 にすると実時間の1/10で運行が終わる。動作確認を早く回したいときに使う。
    実機（SIM_SPEEDUP が無い機体）では何もしない。
    """
    if not speedup:
        return
    if get_param(master, "SIM_SPEEDUP") is None:
        print_step("[警告] SITL ではないため（SIM_SPEEDUP が無い）シミュレーション速度は変更しません。")
        return
    result = set_param(master, "SIM_SPEEDUP", speedup)
    if result is None:
        print_step("[警告] SIM_SPEEDUP を %.0f 倍に設定できませんでした。" % speedup)
    else:
        print_step("シミュレーション速度を %.0f 倍にしました（SIM_SPEEDUP）。" % result)


def apply_speed_params(master, leg):
    """巡航速度を出せるように機体側のパラメータを設定する。

    ミッションの DO_CHANGE_SPEED だけでは速度が上がらないため、機体側の設定も合わせる。
      ローバー / ボート: CRUISE_SPEED, WP_SPEED[m/s]
        ArduPilot Rover はミッション中の速度を CRUISE_SPEED と CRUISE_THROTTLE から
        推定した最大速度（= CRUISE_SPEED / CRUISE_THROTTLE）で制限する。
        既定は CRUISE_SPEED=2, CRUISE_THROTTLE=50% なので 4m/s で頭打ちになり、
        それ以上を DO_CHANGE_SPEED で指定しても速くならない。
      コプター: WP_SPD[m/s]（FWにより WPNAV_SPD[m/s] / WPNAV_SPEED[cm/s]）
        DO_CHANGE_SPEED で指定した速度は、このパラメータの既定値で上書きされてしまう。
    """
    route = leg.route_def

    # コプターの水平速度は最大傾斜角で決まる（既定30度では約10m/sで頭打ち）
    if route.lean_angle_max:
        set_first_existing_param(master, (
            ("ATC_ANGLE_MAX", route.lean_angle_max, "度"),           # ArduPilot 4.7 以降
            ("ANGLE_MAX", route.lean_angle_max * 100.0, "centi度"),  # 4.6 以前
        ))

    speed = route.cruise_speed
    if not speed:
        return

    if leg.needs_takeoff:
        set_first_existing_param(master, (
            ("WP_SPD", speed, "m/s"),                # ArduPilot 4.7 以降(Copter)
            ("WPNAV_SPD", speed, "m/s"),
            ("WPNAV_SPEED", speed * 100.0, "cm/s"),  # 4.6 以前
        ))
    else:
        set_first_existing_param(master, (("CRUISE_SPEED", speed, "m/s"),))
        set_first_existing_param(master, (
            ("WP_SPEED", speed, "m/s"),              # Rover
            ("WP_SPD", speed, "m/s"),
        ))


def write_route_mission(master, leg):
    """routes.py の座標からミッションを生成し、機体へアップロードする。"""
    route = leg.route_def
    items = routes.build_mission(route)
    print_step("ミッションを生成しました: %s（%d アイテム / ルート長 %.0f m / 巡航 %.1f m/s）"
               % (leg.route, len(items), routes.route_length_m(route), route.cruise_speed))
    routes.upload_mission(master, items)
    print_step("ミッションを機体へアップロードしました。")


def download_mission(master, retries=3):
    """機体に書き込まれているミッションをダウンロードして返す。

    アップロードした内容が機体に入ったかの「検証」と、
    「到着判定に使う情報（最終ウェイポイント・離陸高度）の取得」が目的。
    """
    mission_count = None
    for _ in range(retries):
        master.mav.mission_request_list_send(
            master.target_system, master.target_component)
        msg = master.recv_match(type="MISSION_COUNT", blocking=True, timeout=5)
        if msg is not None:
            mission_count = msg.count
            break
    if mission_count is None:
        raise TimeoutError("MISSION_COUNT を受信できませんでした（ミッション数の取得に失敗）。")

    items = []
    for seq in range(mission_count):
        item = None
        for _ in range(retries):
            master.mav.mission_request_int_send(
                master.target_system, master.target_component, seq)
            item = master.recv_match(type="MISSION_ITEM_INT", blocking=True, timeout=5)
            if item is not None and item.seq == seq:
                break
            item = None
        if item is None:
            raise TimeoutError("ミッションアイテム seq=%d を取得できませんでした。" % seq)
        items.append(item)

    # ダウンロード完了を機体へ通知する
    master.mav.mission_ack_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_MISSION_ACCEPTED)

    return items


def verify_mission(leg, items):
    """ミッションが実行可能か検証し、離陸高度・最終ウェイポイント・末尾着陸かを返す。

    seq=0 はホームなので、ホーム以外に1つ以上コマンドが無いと AUTO の init に失敗する
    （機体側が "Mode change to Auto failed: init failed" を出す）。飛ばす前に落とす。
    """
    print_step("機体に入っているミッションアイテム数: %d（seq=0はホーム）" % len(items))
    if len(items) < 2:
        raise RuntimeError(
            "%s にミッションが書き込まれていません（アイテム数=%d）。\n"
            "    AUTOモードへ切り替えられないため中止します。\n"
            "    --no-upload-missions を付けている場合は、MissionPlanner等で\n"
            "    このルートのミッションを機体へ『書き込み』してください。"
            % (leg.name, len(items)))

    # 離陸高度: 先頭の NAV_TAKEOFF の高度[m]を使う（コプターのみ使用）
    takeoff_alt = None
    for item in items:
        if item.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            takeoff_alt = float(item.z)
            break

    # 最終ウェイポイント: 到着判定のフォールバック（距離ベース）に使う
    last_wp = None
    for item in reversed(items):
        if item.command in POSITIONAL_NAV_CMDS and (item.x != 0 or item.y != 0):
            last_wp = (item.x / 1e7, item.y / 1e7)
            break
    if last_wp is not None:
        print_step("最終ウェイポイント: lat=%.7f, lon=%.7f" % last_wp)

    # 末尾が着陸コマンドなら、到着＝着陸完了（ディスアーム）で判定する
    lands_at_goal = items[-1].command == mavutil.mavlink.MAV_CMD_NAV_LAND
    if lands_at_goal:
        print_step("ミッション末尾は着陸(NAV_LAND)です。着陸完了を到着とみなします。")

    return takeoff_alt, last_wp, lands_at_goal


def reset_mission_to_start(master):
    """ミッションの実行位置を先頭に戻す。

    前回の実行でミッションが最後まで進んだままの機体でも、
    再実行時に必ず最初から走らせるため。
    """
    master.mav.mission_set_current_send(
        master.target_system, master.target_component, 0)
    print_step("ミッション実行位置を先頭(seq=0)に戻しました。")


def start_mission(master):
    """MISSION_START を送信し、COMMAND_ACK の結果を表示する。"""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0, 0, 0, 0, 0, 0, 0, 0)

    deadline = time.time() + 5.0
    while time.time() < deadline:
        ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=1)
        if ack is None or ack.command != mavutil.mavlink.MAV_CMD_MISSION_START:
            continue
        result = mavutil.mavlink.enums["MAV_RESULT"][ack.result].name
        print_step("MISSION_START の応答: %s" % result)
        return
    # AUTOモードに入っていれば ACK が無くてもミッションは進むため、警告のみ
    print_step("MISSION_START の応答を受信できませんでした（AUTOモードで進行中なら問題ありません）。")


def takeoff(master, target_alt):
    """コプターを target_alt[m]（対地高度）まで離陸させる。"""
    print_step("離陸します（目標高度 %.1f m）。" % target_alt)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, target_alt)

    # GLOBAL_POSITION_INT(33) を 5Hz で受信して高度を監視する
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5)

    deadline = time.time() + TAKEOFF_TIMEOUT
    last_print = 0.0
    while time.time() < deadline:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
        if msg is None:
            continue
        current_alt = msg.relative_alt / 1000.0
        now = time.time()
        if now - last_print >= 1.0:
            print_step("高度: %.1f m" % current_alt)
            last_print = now
        if current_alt >= target_alt * 0.95:
            print_step("目標高度に到達しました。")
            return

    raise TimeoutError(
        "離陸が %.0f秒以内に完了しませんでした。" % TAKEOFF_TIMEOUT)


# ---------------------------------------------------------------------------
# 到着判定
# ---------------------------------------------------------------------------

def wait_for_arrival(master, leg, items, last_wp, leg_timeout, lands_at_goal=False):
    """機体が目的地に到着するまで待つ。到着理由の文字列を返す。

    lands_at_goal=True（ミッション末尾が着陸）の場合、距離ベースの判定は使わない。
    配送先の真上で降下している最中に「到着」と判定してしまい、着陸完了前に
    こちらからモードを変えてしまうため。
    """
    last_seq = len(items) - 1
    print_step("到着待ち（最終シーケンス番号 seq=%d、最大 %.0f秒）..." % (last_seq, leg_timeout))

    # 到着判定に使うメッセージのレートを要求する
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 2)
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT, 1)

    deadline = time.time() + leg_timeout
    last_beat = 0.0
    last_print = 0.0
    near_since = None
    current_seq = None
    start_position = None   # このレグの出発地点（出発したかの判定に使う）
    departed = False        # 出発地点から DEPARTURE_DISTANCE_M 以上離れたか

    while time.time() < deadline:
        now = time.time()
        if now - last_beat >= 1.0:
            send_heartbeat(master)
            last_beat = now

        msg = master.recv_match(
            type=["MISSION_ITEM_REACHED", "MISSION_CURRENT", "GLOBAL_POSITION_INT",
                  "STATUSTEXT", "HEARTBEAT"],
            blocking=True, timeout=1)
        if msg is None:
            continue
        msg_type = msg.get_type()

        # (1) 最終アイテムへの到達通知（最も確実な判定）
        if msg_type == "MISSION_ITEM_REACHED":
            print_step("ウェイポイント seq=%d に到達" % msg.seq)
            if msg.seq >= last_seq:
                return "最終ウェイポイント(seq=%d)に到達" % msg.seq

        # (2) ミッション状態が「完了」になった（対応FWのみ）
        elif msg_type == "MISSION_CURRENT":
            current_seq = msg.seq
            if getattr(msg, "mission_state", 0) == MISSION_STATE_COMPLETE:
                return "ミッション完了(MISSION_CURRENT.mission_state)"

        # (3) 機体からのミッション完了メッセージ
        elif msg_type == "STATUSTEXT":
            text = msg.text.strip()
            if text:
                print_step("[機体メッセージ] %s" % text)
            lowered = text.lower()
            for keyword in MISSION_DONE_TEXTS:
                if keyword in lowered:
                    return "機体メッセージ '%s'" % text

        # (4) 最終ウェイポイント付近に留まった（到達通知を取りこぼした場合の保険）
        elif msg_type == "GLOBAL_POSITION_INT":
            current_lat = msg.lat / 1e7
            current_lon = msg.lon / 1e7
            if start_position is None:
                start_position = (current_lat, current_lon)
            elif not departed:
                if get_distance_metres(current_lat, current_lon,
                                       start_position[0], start_position[1]) >= DEPARTURE_DISTANCE_M:
                    departed = True

            if last_wp is not None:
                distance = get_distance_metres(current_lat, current_lon, last_wp[0], last_wp[1])
                if now - last_print >= 5.0:
                    print_step("目的地まで %.1f m（実行中 seq=%s）"
                               % (distance, "-" if current_seq is None else current_seq))
                    last_print = now

                # 出発地点が最終ウェイポイントの近くにある場合（前回の運行でそこに着地した等）、
                # 動き出す前に「到着」と誤判定しないよう、次のどちらかを満たすことを条件にする。
                #   - ミッションが最終アイテムを実行中である
                #   - 一度出発地点から DEPARTURE_DISTANCE_M 以上離れた
                heading_to_last = (current_seq is not None and current_seq >= last_seq)
                if not lands_at_goal and distance <= ARRIVE_RADIUS_M and (heading_to_last or departed):
                    if near_since is None:
                        near_since = now
                    elif now - near_since >= ARRIVE_SETTLE_SEC:
                        return "目的地から %.1f m 以内に %.0f秒 留まった" % (
                            ARRIVE_RADIUS_M, ARRIVE_SETTLE_SEC)
                else:
                    near_since = None

        # (5) ディスアームされた（ミッション末尾の LAND / DISARM で自動停止）
        elif msg_type == "HEARTBEAT":
            if not master.motors_armed():
                return "機体がディスアームされた（ミッション終了）"

    raise TimeoutError(
        "%s が %.0f秒以内に目的地へ到着しませんでした。" % (leg.name, leg_timeout))


def finish_leg(master, leg, keep_copter_airborne):
    """到着後の処理。荷物を載せ替えられる状態（停止・ディスアーム）にする。"""
    if leg.needs_takeoff:
        # コプター: ミッション末尾が LAND なら自動で降りているので、その場合は待つだけ
        if not master.motors_armed():
            print_step("着陸・ディスアーム済みです。")
            return
        if keep_copter_airborne:
            print_step("--keep-copter-airborne 指定のため、着陸させずに処理を終了します。")
            return
        print_step("LANDモードへ切り替えて着陸します。")
        set_mode(master, "LAND")
        if wait_disarmed(master, timeout=180.0):
            print_step("着陸・ディスアームを確認しました。")
        else:
            print_step("[警告] 着陸後のディスアームを確認できませんでした。機体の状態を確認してください。")
        return

    # ローバー / ボート: HOLD で停止させてディスアーム
    if master.motors_armed():
        set_mode(master, "HOLD")
        disarm_vehicle(master)
    else:
        print_step("既にディスアーム済みです。")


# ---------------------------------------------------------------------------
# 荷物の載せ替え
# ---------------------------------------------------------------------------

def wait_cargo_transfer(seconds, confirm, work_name="荷物の載せ替え"):
    """荷物の載せ替えを待つ（回送モードでは次の機体の出発準備の待ち時間になる）。

    confirm=True の場合はオペレーターの Enter 入力を待つ（実運用向け）。
    そうでない場合は seconds 秒のカウントダウンで作業を模擬する。
    """
    print_banner("%s中" % work_name)
    if confirm:
        input("  %sが完了したら Enter を押してください（次の機体が出発します）: " % work_name)
        print("  %s完了。次の機体を出発させます。" % work_name)
        return

    remaining = float(seconds)
    while remaining > 0:
        sys.stdout.write("\r  %s 残り %4.1f 秒 ..." % (work_name, remaining))
        sys.stdout.flush()
        time.sleep(0.5)
        remaining -= 0.5
    sys.stdout.write("\r  %s完了。次の機体を出発させます。      \n" % work_name)
    sys.stdout.flush()


# ---------------------------------------------------------------------------
# レグの実行
# ---------------------------------------------------------------------------

def run_leg(leg, args, index, total, upload_mission=False):
    """1区間を運行する。所要時間[秒]を返す。"""
    print_banner("[レグ %d/%d] %s : %s" % (index, total, leg.name, leg.route))

    master = connect_vehicle(args.host, leg.port, args.connect_timeout)
    try:
        # 出発地に居ることを確認する（事前準備で配置済みなら何もしない）
        if args.set_start_position:
            master = ensure_start_position(master, leg, args)

        # 事前チェックを省略した場合は、ここでミッションを生成・アップロードする
        if upload_mission:
            write_route_mission(master, leg)

        # アップロードした（または手動で書き込まれた）ミッションを読み戻して検証する
        items = download_mission(master)
        takeoff_alt, last_wp, lands_at_goal = verify_mission(leg, items)
        apply_sim_speedup(master, args.speedup)
        apply_speed_params(master, leg)
        reset_mission_to_start(master)

        # 所要時間は「出発（アーム）から到着まで」を測る（初期位置の設定時間は含めない）
        started_at = time.time()

        if leg.needs_takeoff:
            # コプター: アーム → 離陸 → ミッション開始
            set_mode(master, leg.arm_mode)          # GUIDED
            arm_vehicle(master)
            alt = takeoff_alt if takeoff_alt else args.takeoff_alt
            if takeoff_alt:
                print_step("ミッションの NAV_TAKEOFF から離陸高度 %.1f m を使用します。" % alt)
            else:
                print_step("ミッションに NAV_TAKEOFF が無いため、既定の離陸高度 %.1f m を使用します。" % alt)
            takeoff(master, alt)
            set_mode(master, "AUTO")
            start_mission(master)
        else:
            # ローバー / ボート: アーム → ミッション開始
            set_mode(master, leg.arm_mode)          # HOLD（アーム時の暴走防止）
            arm_vehicle(master)
            set_mode(master, "AUTO")
            start_mission(master)

        print_step("ミッション実行中。目的地への到着を待ちます。")
        reason = wait_for_arrival(master, leg, items, last_wp, args.leg_timeout,
                                  lands_at_goal=lands_at_goal)
        elapsed = time.time() - started_at
        print_step("到着を検知しました（%s） 所要時間 %.1f 秒" % (reason, elapsed))

        finish_leg(master, leg, args.keep_copter_airborne)
        return elapsed
    finally:
        master.close()


def precheck(legs, args):
    """出発前に、3機すべてへミッションを書き込み、実行可能かを確認する。

    2番目・3番目の機体のミッションに問題があることに運行開始後に気づくと、
    先頭の機体だけが目的地に取り残されるため、走らせる前にまとめて処理する。
    """
    if args.upload_missions:
        print_banner("事前準備: 全機体にミッションを生成・アップロードします")
    else:
        print_banner("事前チェック: 全機体の接続とミッションを確認します")

    for leg in legs:
        print("\n  ● %s (%s)" % (leg.name, leg.route))
        master = connect_vehicle(args.host, leg.port, args.connect_timeout)
        try:
            # 出発地に配置してからミッションを書き込む（ホームが出発地になる）
            if args.set_start_position:
                master = ensure_start_position(master, leg, args)
            if args.upload_missions:
                write_route_mission(master, leg)
            items = download_mission(master)
            verify_mission(leg, items)
            if master.motors_armed():
                print_step("[警告] この機体は既にアーム済みです。")
        finally:
            master.close()
    print("\n  事前準備完了。すべての機体が運行可能です。")


def parse_args():
    parser = argparse.ArgumentParser(
        description="ローバー → ボート → コプターのリレー運行（前の機体の到着・載せ替え後に次が出発）")
    parser.add_argument("--host", default=None,
                        help="機体（SITL/MissionPlanner）のホスト（既定: %s。"
                             "--mission-planner 指定時は Windows 側のIPを自動判定）" % DEFAULT_HOST)
    parser.add_argument("--mission-planner", action="store_true",
                        help="Mission Planner でSITLを起動した構成に合わせる"
                             "（ポートを 5762/5772/5782 にし、WSLからはWindows側のIPへ接続する）")
    parser.add_argument("--no-autodetect", dest="autodetect",
                        action="store_false", default=True,
                        help="接続先（ホスト・ポート）の自動判定を行わない")
    parser.add_argument("--rover-port", type=int, default=None,
                        help="ローバーのTCPポート（既定: %d、--mission-planner 時 %d）"
                             % (DEFAULT_ROVER_PORT, DEFAULT_ROVER_PORT + 2))
    parser.add_argument("--boat-port", type=int, default=None,
                        help="ボートのTCPポート（既定: %d、--mission-planner 時 %d）"
                             % (DEFAULT_BOAT_PORT, DEFAULT_BOAT_PORT + 2))
    parser.add_argument("--copter-port", type=int, default=None,
                        help="コプターのTCPポート（既定: %d、--mission-planner 時 %d）"
                             % (DEFAULT_COPTER_PORT, DEFAULT_COPTER_PORT + 2))
    parser.add_argument("--transfer-sec", type=float, default=DEFAULT_TRANSFER_SEC,
                        help="荷物の載せ替えに要する時間[秒]（既定: %.0f）" % DEFAULT_TRANSFER_SEC)
    parser.add_argument("--confirm", action="store_true",
                        help="載せ替え完了をオペレーターの Enter 入力で判断する")
    parser.add_argument("--speedup", type=float, default=None,
                        help="SITLのシミュレーション速度[倍]（例: 10 で10倍速。既定は機体の設定のまま）")
    parser.add_argument("--connect-timeout", type=float, default=DEFAULT_CONNECT_TIMEOUT,
                        help="機体への接続（起動待ち）のタイムアウト[秒]（既定: %.0f）"
                             % DEFAULT_CONNECT_TIMEOUT)
    parser.add_argument("--leg-timeout", type=float, default=DEFAULT_LEG_TIMEOUT,
                        help="1レグの到着待ちタイムアウト[秒]（既定: %.0f）" % DEFAULT_LEG_TIMEOUT)
    parser.add_argument("--takeoff-alt", type=float, default=DEFAULT_TAKEOFF_ALT,
                        help="ミッションに NAV_TAKEOFF が無い場合の離陸高度[m]（既定: %.0f）"
                             % DEFAULT_TAKEOFF_ALT)
    parser.add_argument("--keep-copter-airborne", action="store_true",
                        help="コプターの到着後に自動着陸させない（ミッション末尾で着陸する場合等）")
    parser.add_argument("--skip-precheck", action="store_true",
                        help="出発前の全機体まとめての準備・チェックを省略する"
                             "（ミッションのアップロードは各レグの開始時に行う）")
    parser.add_argument("--legs", default=None,
                        help="運行する機体をカンマ区切りで指定（rover,boat,copter）。"
                             "省略時は3機すべてをリレー運行する（動作確認用）")
    parser.add_argument("--return", dest="return_flight", action="store_true",
                        help="回送に固定する。配送先にいる機体を、コプター → ボート → ローバー の順に"
                             "各ルートを逆向きに運行して出発地へ戻す"
                             "（既定は機体の位置から自動判定）")
    parser.add_argument("--delivery", dest="force_delivery", action="store_true",
                        help="配送に固定する（既定は機体の位置から自動判定）")
    parser.add_argument("--no-upload-missions", dest="upload_missions",
                        action="store_false", default=True,
                        help="ミッションを生成・アップロードしない"
                             "（Mission Planner等で書き込んだミッションを使う）")
    parser.add_argument("--no-set-start-position", dest="set_start_position",
                        action="store_false", default=True,
                        help="各機体が出発地にいるかの確認を行わない")
    parser.add_argument("--reposition-by-reboot", action="store_true",
                        help="出発地から離れている機体を、SITLの初期位置を設定して再起動することで"
                             "出発地に戻す（Mission Planner が起動したSITLでは追加ポートが"
                             "失われるため非推奨。sim_vehicle.py で起動したSITL向け）")
    parser.add_argument("--start-tolerance", type=float, default=DEFAULT_START_TOLERANCE,
                        help="出発地に居るとみなす距離[m]（既定: %.0f）" % DEFAULT_START_TOLERANCE)
    parser.add_argument("--export-waypoints", metavar="DIR", default=None,
                        help="生成したミッションを DIR に .waypoints 形式で書き出して終了する"
                             "（Mission Planner で内容を確認できる）")
    return parser.parse_args()


def print_route_summary(args, legs):
    """これから運行するルート一覧を表示する。"""
    if args.return_flight:
        print_banner("複数機体の回送（コプター → ボート → ローバー）")
        print("  配送先にいる機体を、各ルートを逆向きに運行して出発地へ戻します。")
    else:
        print_banner("複数機体リレー運行（ローバー → ボート → コプター）")
    print("  接続先ホスト: %s%s" % (args.host,
                                "  (--mission-planner)" if args.mission_planner else ""))
    for i, leg in enumerate(legs, start=1):
        print("  %d. %s port=%d  %s  (%.0f m)"
              % (i, pad(leg.name, 12), leg.port, pad(leg.route, 32),
                 routes.route_length_m(leg.route_def)))
    if args.upload_missions:
        print("  ミッションは routes.py の座標からプログラムが生成して各機体に書き込みます。")
    else:
        print("  ミッションは各機体に書き込み済みである前提です（--no-upload-missions）。")


def main():
    args = parse_args()
    resolve_direction(args)
    resolve_connection_settings(args)
    legs = build_legs(args)

    # --export-waypoints: ミッションをファイルに書き出すだけで終了する（機体に接続しない）
    if args.export_waypoints:
        print_route_summary(args, legs)
        print_banner("ミッションを .waypoints 形式で書き出します")
        os.makedirs(args.export_waypoints, exist_ok=True)
        for leg in legs:
            path = os.path.join(args.export_waypoints,
                                "mission_%s%s.waypoints"
                                % (leg.route_def.key, "_return" if args.return_flight else ""))
            routes.export_waypoints(leg.route_def, path)
            print_step("%s → %s" % (leg.route, path))
        return

    try:
        # 指定の host/ポートで MAVLink が流れていない場合は、応答する組み合わせを探す
        if args.autodetect:
            autodetect_connection(args, legs)

        # 配送か回送かを、各機体が出発地にいるかどうかで決める
        if args.direction == "auto":
            args.return_flight = decide_direction(args, legs)
            legs = build_legs(args)

        print_route_summary(args, legs)

        if not args.skip_precheck:
            precheck(legs, args)

        results = []
        for i, leg in enumerate(legs, start=1):
            # 事前準備を省略した場合は、各レグの開始時にアップロードする
            elapsed = run_leg(leg, args, i, len(legs),
                              upload_mission=args.upload_missions and args.skip_precheck)
            results.append((leg, elapsed))

            # 最後のレグの後は待たない（載せ替える相手／次に出す機体がいない）
            if i < len(legs):
                wait_cargo_transfer(args.transfer_sec, args.confirm,
                                    "次の機体の出発準備" if args.return_flight
                                    else "荷物の載せ替え")

        print_banner("全機体の回送が完了しました" if args.return_flight
                     else "全ルートの運行が完了しました")
        total = 0.0
        for leg, elapsed in results:
            print("  %s %s %6.1f 秒" % (pad(leg.name, 12), pad(leg.route, 34), elapsed))
            total += elapsed
        print("  ----")
        print("  飛行・走行時間の合計: %.1f 秒（レグ間の待ち時間は含みません）" % total)

    except KeyboardInterrupt:
        print("\n手動で中断されました。")
        print("機体が動作中の場合は、MissionPlanner等から HOLD / RTL / LAND で安全に停止させてください。")
        sys.exit(130)

    except (TimeoutError, RuntimeError, OSError) as e:
        print("\n[エラー] %s" % e)
        print("運行を中止しました。機体が動作中の場合は、"
              "MissionPlanner等から HOLD / RTL / LAND で安全に停止させてください。")
        sys.exit(1)


if __name__ == "__main__":
    main()
