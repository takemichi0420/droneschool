# -*- coding: utf-8 -*-
"""
全自動フライト + 区間別バッテリー消費量計測スクリプト

WayPointファイルで作成した、上昇、下降、南、西、北、東の四角形ミッションを自動で実行し、
各区間ごとのバッテリー消費量(mAh)を計測する
※専用に作成したWayPointファイルが必要。 機体の向きは、常に北向きで飛行するようにしている

前提:
  - 計測開始、終了のトリガは、サーボ9番のPWM信号を1900と1100に変化させることで本プログラム側から検知する
    よって、サーボ9番が未使用チャンネルであること。使用済の場合、他の未使用チャンネルに変更し、
    WayPointファイルのCMD:183行の値とそろえること
  - 予め本飛行プログラム専用に作成した.waypointファイルは、実行環境と同一のカレントディレクトリに配置する
    WayPointファイルは、ARM前に自動で書き込みされる
  - GPSフィックス、EKF、コンパス等の事前チェックに問題がないこと

安全対策
  - ARM前にバッテリー残量が20%未満の場合、ARM/離陸せずに終了する
  - バッテリー残量が20%未満になった場合、強制的にRTLする（RTL_ALTITUDE[15m]まで上昇して帰還）
  - 電圧ベースの保険（残量%が取得できない機体への保険）として、起動時にオペレーターが手動でセル数を入力し、
    LOW_CELL_VOLTAGE[V/セル]を下回った場合に強制RTLする

流れ:
  1. 接続、バッテリーセル数の手動入力（3Sデフォルト）
  2. アーミング可能になるまで待機
  3. GUIDEDモードでARM
     -> ARM後、WayPointファイルNo.0行(seq=0)の緯度経度を強制的にホームポイントに設定
        （MissionPlannerでの機体の現在位置に関わらず、No.0の緯度経度をホームとする）
        高度はホーム設定時の機体の実測値（GPS絶対高度）を使用する
  4. ARM完了後、コンソールで離陸(ミッション実行)の可否を確認
  5. 許可されたら、まずGUIDEDモードで垂直に10m上昇（低高度での水平移動を避ける安全対策）
     -> その後AUTOモードに切替
          
     以下は WayPointファイルの内容に依存する
     -> 10mで5秒待機
     -> 区間1: 上昇 (10m -> 25m)
     -> 区間2: 下降 (25m -> 10m)
     -> 区間3〜7: 既存の四角形ミッション各辺15m（各脚に開始/終了マーカー）
     -> 最終脚でホームポイント上空に帰還
     -> LOITER_UNLIM でホーム上空にてホバリング待機（自動着陸はしない）

  6. 飛行中、servo9_raw の値変化を監視して区間ごとのバッテリー消費量(mAh)を記録
     区間が終了するたびに CSV を都度保存（ホーム上空帰還時点でも確実に記録される）
  7. 全区間（TOTAL_LEGS区間）の計測が完了したら、コンソールで着陸の実行可否を確認
  8. 確認が取れたら LAND モードに切り替えて着陸、ディスアームを検知して終了
"""

from dronekit import connect, VehicleMode, LocationGlobal, Command
import time
import csv
import datetime
import sys
import select
import math
import os
import glob

# ==== 接続設定 ====
# ※※※接続方法毎に変更※※※
#
# WSL 上の SITL(sim_vehicle.py) + Windows の Mission Planner 併用時:
CONNECTION_STRING = "tcp:127.0.0.1:5762"
# Windowsから起動したMission Plannerの場合
#CONNECTION_STRING = "tcp:192.168.3.210:5762"  # Windows側のIPアドレスに変更すること
#MAVProxy が 14551 にも --out する構成の場合
#CONNECTION_STRING = "udp:127.0.0.1:14551"      
#BAUD = 57600

# ==== WayPointファイル設定 ====
# No.0行(seq=0)の座標を強制的にホームポイントに設定するために参照する。
# 機体をMissionPlanner等でどこにいる状態でも、このファイルのNo.0座標をホームにする。
# 実行時のカレントディレクトリにある .waypoints ファイルを参照する。
def find_waypoint_file():
    """
    実行時のカレントディレクトリ内の .waypoints ファイルを探して返す。
    参照するファイルを一意に決めるため、見つからない場合・複数ある場合はエラーにする。
    """
    candidates = sorted(glob.glob(os.path.join(os.getcwd(), "*.waypoints")))
    if not candidates:
        raise FileNotFoundError(
            "カレントディレクトリに .waypoints ファイルが見つかりません: %s" % os.getcwd())
    if len(candidates) > 1:
        raise ValueError(
            "カレントディレクトリに .waypoints ファイルが複数あります。参照するファイルを1つにしてください:\n  %s"
            % "\n  ".join(candidates))
    return candidates[0]


WAYPOINT_FILE = find_waypoint_file()

# ==== バッテリー計測トリガー設定（WPファイルの CMD:183と合わせる） ====
SERVO_CHANNEL = 9           # ※※※空きチャンネルにすること※※※
PWM_THRESHOLD = 1500        # これを超えたら開始、下回ったら終了

# Square11.waypoints の区間数（上昇・下降・南西北東の4方向 = 6）
TOTAL_LEGS = 6

# バッテリー残量がこの値[%]未満になったら強制的にRTLする
LOW_BATTERY_THRESHOLD = 20

# 強制RTL時に上昇する高度[m]（ArduPilotのRTL_ALTパラメータに設定する）
RTL_ALTITUDE = 15

# AUTO(ミッション)開始前に、GUIDEDモードで垂直に上昇する高度[m]
# WayPoint先頭座標と現在地が離れていても、低高度での水平移動を避けるため一旦真上へ上昇する
TAKEOFF_ALTITUDE = 10.0   ##### デモ時は低めにする　　実運用では周囲の建物の高さ以上確保
# 垂直離陸の完了待ちタイムアウト[秒]
TAKEOFF_TIMEOUT = 30.0

# 着陸確認前に、ホームポイント上空への到達とみなす水平距離[m]
HOME_ARRIVAL_RADIUS = 3.0
# ホーム到達待ちのタイムアウト[秒]（超過しても着陸確認へ進む）
HOME_ARRIVAL_TIMEOUT = 120.0

# ==== 電圧ベースの保険RTL設定（残量%が取得できない機体への保険） ====
# セル数はARM前にオペレーターが手動入力する。空入力（space/Enter）時の既定セル数。
DEFAULT_CELL_COUNT = 3
# このセル電圧[V/セル]を下回ったら強制RTL（下限電圧 = 入力セル数 × この値）
LOW_CELL_VOLTAGE = 3.4
# 電圧は負荷変動(サグ)で瞬間的に落ちるため、この秒数連続で下回った場合のみ発動する
LOW_VOLTAGE_TIMER = 5.0

# ファイル名: battery_consumed_YYYYMMDD_HHMM.csv（実行時の日付・時刻を使用）
OUTPUT_CSV = "battery_consumed_%s.csv" % datetime.datetime.now().strftime("%Y%m%d_%H%M")


def wait_until_armable(vehicle):
    print("機体の状態確認中（GPS/EKF/コンパス等）...")
    while not vehicle.is_armable:
        print("  is_armable=False ... 待機中 (GPS: %s, EKF OK: %s)" % (
            vehicle.gps_0.fix_type, vehicle.ekf_ok))
        time.sleep(1)
    print("アーミング可能な状態になりました。")


def set_param_safe(vehicle, name, value, retries=5, wait=2.0, tol=None):
    """
    パラメータを設定し、実際に反映されたか読み戻して検証する。

    DroneKitは PARAM_SET 送信後に PARAM_VALUE 応答を待つが、応答の取りこぼしで
    'timeout setting parameter' を出しても、値自体は機体に反映されていることが多い。
    そこで固定sleep後に1回だけ読むのではなく、設定→数秒ポーリングで読み戻し、
    目標値に一致するまでリトライする。

    - 機体に存在しないパラメータ名の場合は無応答でタイムアウトするため、
      事前に存在チェックし、無ければ明確に警告して None を返す。
    戻り値: 反映が確認できた最終値(float)。存在しない/未反映の場合は None。
    """
    value = float(value)
    if tol is None:
        # 整数系(ビットマスク等)は完全一致、実数系は微小許容
        tol = 0.0 if value.is_integer() else 0.5

    # 存在チェック（ダウンロード直後でキャッシュ未反映の可能性も考慮して少し待つ）
    deadline = time.time() + wait
    while vehicle.parameters.get(name, None) is None and time.time() < deadline:
        time.sleep(0.3)
    if vehicle.parameters.get(name, None) is None:
        print("  [%s] このパラメータは機体に存在しません。パラメータ名を確認してください"
              "（ファーム更新で改名された可能性）。" % name)
        return None

    for attempt in range(1, retries + 1):
        try:
            vehicle.parameters[name] = value
        except Exception as e:
            print("  [%s] 設定送信で例外: %s（試行 %d/%d）" % (name, e, attempt, retries))
        deadline = time.time() + wait
        while time.time() < deadline:
            cur = vehicle.parameters.get(name, None)
            if cur is not None and abs(float(cur) - value) <= tol:
                return float(cur)
            time.sleep(0.3)
        print("  [%s] 反映を確認できず再試行します（試行 %d/%d, 現在値=%s）。" % (
            name, attempt, retries, vehicle.parameters.get(name, None)))

    return vehicle.parameters.get(name, None)


def ensure_auto_options(vehicle):
    """
    ArduPilot Copterはデフォルトで、AUTOモード中に地上からミッションを開始する際、
    RC送信機のスロットルを上げないと離陸(ミッション)が始まらない安全設計になっている。
    DroneKit(MAVLink)のみで完全自動運用する場合、このチェックが永久に満たされず
    「ARM済み・AUTOモードだが離陸しない」状態になるため、AUTO_OPTIONSパラメータで
    この挙動を無効化する。
      bit0 (1) = AUTOモード中のアーミングを許可
      bit1 (2) = スロットルを上げなくても離陸(ミッション)を開始することを許可
    """

    current = vehicle.parameters.get('AUTO_OPTIONS', 0)
    required_bits = 0b11  # bit0 + bit1 = 3
    new_value = int(current) | required_bits

    if int(current) != new_value:
        print("AUTO_OPTIONS を %d -> %d に変更します（スロットル操作なしでの自動離陸を許可）。" % (
            int(current), new_value))
        result = set_param_safe(vehicle, 'AUTO_OPTIONS', new_value)
        if result is not None:
            print("AUTO_OPTIONS 設定後の値: %d" % int(result))
        else:
            print("[警告] AUTO_OPTIONS を設定できませんでした。スロットル操作なしでは"
                  "自動離陸しない可能性があります。")
    else:
        print("AUTO_OPTIONS は既に必要なビットが立っています（値=%d）。" % int(current))


def ensure_rtl_altitude(vehicle):
    """
    強制RTL時の上昇高度を RTL_ALTITUDE[m] に設定する。

    ArduPilot 4.6 以降はパラメータ名/単位が刷新され、RTL高度は
      RTL_ALT_M （単位: m）
    になった。旧ファームは
      RTL_ALT   （単位: cm）
    のため、機体に存在する方を自動判別して、それぞれの単位で設定する。
    """
    # 新FW(RTL_ALT_M, m単位)を優先。無ければ旧FW(RTL_ALT, cm単位)。
    if vehicle.parameters.get('RTL_ALT_M', None) is not None:
        name, target, unit = 'RTL_ALT_M', float(RTL_ALTITUDE), 'm'
    else:
        name, target, unit = 'RTL_ALT', float(RTL_ALTITUDE * 100), 'cm'

    current = vehicle.parameters.get(name, None)
    if current is not None and abs(float(current) - target) < 0.5:
        print("%s は既に %s%s (=%dm) に設定されています。" % (name, target, unit, RTL_ALTITUDE))
        return

    print("%s を %s -> %s%s (=%dm) に設定します。" % (
        name, current, target, unit, RTL_ALTITUDE))
    result = set_param_safe(vehicle, name, target)
    if result is not None:
        print("%s 設定後の値: %s (%s)" % (name, result, unit))
    else:
        print("[警告] %s を設定できませんでした。強制RTL時の上昇高度が"
              "意図通りにならない可能性があります。" % name)


def read_home_from_waypoints(path):
    """
    WayPointファイル(QGC WPL 110形式)のNo.0行(seq=0)から
    ホームポイントの緯度・経度・高度を読み取って返す。
    フィールドはタブ区切りで、seq lat lon alt はそれぞれ 0,8,9,10列目。
    """
    with open(path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    # 1行目はヘッダ("QGC WPL 110")なので2行目以降を走査
    for line in lines[1:]:
        parts = line.strip().split("\t")
        if len(parts) < 12:
            continue
        if int(parts[0]) == 0:  # seq == 0 がホーム行
            lat = float(parts[8])
            lon = float(parts[9])
            alt = float(parts[10])
            return lat, lon, alt

    raise ValueError("WayPointファイルにNo.0行(seq=0)が見つかりません: %s" % path)


def set_home_location(vehicle, lat, lon):
    """
    ホームポイントの緯度・経度をWayPointファイルNo.0行の座標に強制設定する。
    高度は機体の実測値（GPSの絶対高度[m MSL]）を使用する。
    ArduPilotは通常アーミング時の現在地をホームにするため、機体の実際の位置に
    関わらず指定座標をホームにしたい場合に使用する（DO_SET_HOME相当）。
    アーミング時の自動上書きを避けるため、ARM後に呼び出すこと。
    """
    # home_location はコマンドをダウンロードしないと取得/設定できない
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()

    # 高度は機体の実測値（絶対高度）を使用する
    alt = vehicle.location.global_frame.alt
    if alt is None:
        raise ValueError("機体の高度(実測値)を取得できませんでした。")

    print("ホームポイントを設定します（緯度経度=WayPoint No.0, 高度=機体実測値）: "
          "lat=%.8f, lon=%.8f, alt=%.2f" % (lat, lon, alt))
    vehicle.home_location = LocationGlobal(lat, lon, alt)

    # 反映を待って読み戻す
    time.sleep(2)
    cmds.download()
    cmds.wait_ready()
    home = vehicle.home_location
    if home is not None:
        print("設定後のホームポイント: lat=%.8f, lon=%.8f, alt=%.2f" % (
            home.lat, home.lon, home.alt))
    else:
        print("ホームポイントの読み戻しに失敗しましたが、設定コマンドは送信済みです。")


def get_distance_metres(loc1, loc2):
    """2点の global_frame（緯度経度）間の水平距離[m]を近似計算する。"""
    dlat = loc2.lat - loc1.lat
    dlon = loc2.lon - loc1.lon
    # 緯度1度≒1.113195e5[m]の近似（数百m規模の距離では十分な精度）
    return math.sqrt((dlat * dlat) + (dlon * dlon)) * 1.113195e5


def wait_until_home(vehicle, state):
    """
    機体がホームポイント上空（水平距離 HOME_ARRIVAL_RADIUS[m] 以内）に
    到達するまで待機する。着陸確認プロンプトを出す前に呼び出す。
      - 低バッテリー強制RTLが発動した場合は即座に戻る（呼び出し側でRTL処理へ）
      - ホーム位置が取得できない/タイムアウトの場合は待機を打ち切って戻る
    """
    # home_location は commands.download() 後に得られる
    if vehicle.home_location is None:
        cmds = vehicle.commands
        cmds.download()
        cmds.wait_ready()
    home = vehicle.home_location
    if home is None:
        print("ホーム位置を取得できませんでした。到達判定をスキップして着陸確認へ進みます。")
        return

    print("ホームポイント上空への到達を待機します...")
    wait_start = time.time()
    while True:
        if state["low_battery_rtl"]:
            return
        if time.time() - wait_start > HOME_ARRIVAL_TIMEOUT:
            print("ホーム到達待ちがタイムアウトしました。着陸確認へ進みます。")
            return
        current = vehicle.location.global_frame
        if current is None or current.lat is None or current.lon is None:
            time.sleep(1)
            continue
        dist = get_distance_metres(current, home)
        print("  ホームまで %.1f m" % dist)
        if dist <= HOME_ARRIVAL_RADIUS:
            print("ホームポイント上空に到達しました。")
            return
        time.sleep(1)


def check_battery_before_arm(battery):
    """
    ARM前にバッテリー残量[%]を確認する。
    残量%が LOW_BATTERY_THRESHOLD 未満の場合は False を返し、離陸させない。
    残量%が取得できない機体では電圧ベースの保険で保護するため、
    残量%が不明（None）の場合はチェックをスキップして True を返す。
    """
    # BATTERY_STATUS がまだ届いていない可能性があるため、少しだけ受信を待つ
    wait_start = time.time()
    while battery["remaining"] is None and battery["voltage"] is None:
        if time.time() - wait_start > 5:
            print("バッテリー情報を取得できませんでした。残量%チェックはスキップします。")
            return True
        time.sleep(0.5)

    remaining = battery["remaining"]
    if remaining is None:
        print("バッテリー残量%が取得できない機体のため、ARM前の残量%チェックはスキップします。"
              "（電圧ベースの保険で保護します）")
        return True

    print("ARM前バッテリー残量チェック: 残量 %d%%（しきい値 %d%%）" % (
        remaining, LOW_BATTERY_THRESHOLD))
    if remaining < LOW_BATTERY_THRESHOLD:
        print("バッテリー残量が %d%% 未満（現在 %d%%）のため、ARM/離陸を中止します。" % (
            LOW_BATTERY_THRESHOLD, remaining))
        return False
    return True


def arm_vehicle(vehicle):
    print("モードを GUIDED に設定します。")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        time.sleep(0.5)

    print("ARMします。")
    vehicle.armed = True
    arm_timeout = 30  # ARM完了までの最大待機時間[秒]
    arm_start = time.time()
    while not vehicle.armed:
        if time.time() - arm_start > arm_timeout:
            raise TimeoutError("ARMが%d秒以内に完了しませんでした。" % arm_timeout)
        print("  ARM待機中...")
        time.sleep(1)
    print("ARMしました。")


def guided_takeoff(vehicle, target_alt):
    """
    GUIDEDモードで垂直に target_alt[m](対地高度)まで上昇する。
    WayPoint先頭座標と現在地が離れている場合に、低高度での水平移動で人物等へ
    接触するのを避けるため、AUTO(ミッション)開始前に真上へ離陸させる。
    """
    # simple_takeoff はGUIDEDモード・ARM済みが前提
    if vehicle.mode.name != "GUIDED":
        print("モードを GUIDED に設定します。")
        vehicle.mode = VehicleMode("GUIDED")
        while vehicle.mode.name != "GUIDED":
            time.sleep(0.5)

    print("GUIDEDモードで垂直に %.1fm まで離陸します。" % target_alt)
    vehicle.simple_takeoff(target_alt)

    # 目標高度の95%に達するまで待機（タイムアウトあり）
    takeoff_start = time.time()
    while True:
        alt = vehicle.location.global_relative_frame.alt
        if alt is not None:
            print("  高度 %.1f m" % alt)
            if alt >= target_alt * 0.95:
                print("目標高度に到達しました。")
                return
        if time.time() - takeoff_start > TAKEOFF_TIMEOUT:
            print("垂直離陸が%.0f秒以内に完了しませんでした（現在高度 %s m）。処理を継続します。" % (
                TAKEOFF_TIMEOUT, alt))
            return
        time.sleep(1)


def read_mission_from_waypoints(path):
    """
    WayPointファイル(QGC WPL 110形式)を読み、DroneKitのCommandリストに変換する。
    各行はタブ区切りで
      seq current frame command p1 p2 p3 p4 lat(x) lon(y) alt(z) autocontinue
    の12列。1行目はヘッダ("QGC WPL 110")。seq=0行(ホーム)も含めて読み込む
    （アップロード時に機体側がseqを振り直し、seq0はホームとして扱われる）。
    """
    missionlist = []
    with open(path, "r", encoding="utf-8") as f:
        for i, line in enumerate(f):
            line = line.strip()
            if i == 0:
                if not line.startswith("QGC WPL 110"):
                    raise ValueError(
                        "対応していないWayPointファイル形式です"
                        "（1行目が 'QGC WPL 110' ではありません）: %s" % path)
                continue
            if not line:
                continue
            arr = line.split("\t")
            if len(arr) < 12:
                continue
            ln_current = int(arr[1])
            ln_frame = int(arr[2])
            ln_command = int(arr[3])
            ln_param1 = float(arr[4])
            ln_param2 = float(arr[5])
            ln_param3 = float(arr[6])
            ln_param4 = float(arr[7])
            ln_lat = float(arr[8])
            ln_lon = float(arr[9])
            ln_alt = float(arr[10])
            ln_autocontinue = int(arr[11])
            # Command(target_system, target_component, seq, frame, command, current,
            #         autocontinue, param1..4, x(lat), y(lon), z(alt))
            cmd = Command(0, 0, 0, ln_frame, ln_command, ln_current, ln_autocontinue,
                          ln_param1, ln_param2, ln_param3, ln_param4,
                          ln_lat, ln_lon, ln_alt)
            missionlist.append(cmd)
    return missionlist


def upload_mission(vehicle, path):
    """
    WayPointファイルを機体に書き込む（Mission Planner の「書き込み(Write)」相当）。
    既存ミッションを消去してからファイルの内容をアップロードする。
    ARM前・離陸前に呼び出すこと（AUTO切替時の 'init failed' を根本的に防ぐ）。
    """
    missionlist = read_mission_from_waypoints(path)
    if not missionlist:
        raise ValueError("WayPointファイルに有効なコマンド行がありません: %s" % path)

    cmds = vehicle.commands
    print("WayPointファイルを機体に書き込みます: %s（%d 項目）" % (path, len(missionlist)))
    cmds.clear()
    for cmd in missionlist:
        cmds.add(cmd)
    cmds.upload()   # 送信完了までブロックする
    print("WayPointファイルの書き込みを送信しました。")


def verify_mission_loaded(vehicle):
    """
    機体にミッションが書き込まれているかをARM前に検証する。

    AUTOモードの init は mission.num_commands() > 1（＝ホーム以外に最低1コマンド）
    でないと失敗し、機体側が "Mode change to Auto failed: init failed" を出す。
    DroneKitの vehicle.commands.count はホーム(seq=0)を含まないコマンド数なので、
    count >= 1 であればAUTO実行に足るミッションが存在する。
    離陸してから初めて気づくと危険なので、ARM前にここで確認して落とす。
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    count = cmds.count
    print("機体に書き込まれているミッションコマンド数（ホーム除く）: %d" % count)
    if count < 1:
        raise ValueError(
            "機体にミッションが書き込まれていません（コマンド数=%d）。\n"
            "  AUTOモードへ切り替えられないため中止します。\n"
            "  Mission Planner等で WayPoint を機体に『書き込み』済みか、\n"
            "  接続先(%s)が書き込んだ機体と同一か確認してください。" % (count, CONNECTION_STRING))
    return count


def switch_to_auto(vehicle, timeout=10.0):
    print("モードを AUTO に切り替えます。")
    vehicle.mode = VehicleMode("AUTO")
    # AUTO init が失敗するとモードはGUIDEDのまま戻る。永久ループを避けるため
    # タイムアウトで失敗を検知し、明確に例外を投げる（機体側は "init failed" を出す）。
    deadline = time.time() + timeout
    while vehicle.mode.name != "AUTO":
        if time.time() > deadline:
            raise RuntimeError(
                "AUTOモードへの切り替えに失敗しました（現在モード=%s, %.0f秒待機）。\n"
                "  機体側の 'Mode change to Auto failed: init failed' は、通常\n"
                "  ミッション未書き込み、または地上ARM済みでミッション先頭が離陸コマンド"
                "でない場合に発生します。" % (vehicle.mode.name, timeout))
        time.sleep(0.5)
    print("AUTOモードに入りました。ミッションを実行します。")


def disarm_vehicle(vehicle):
    print("ディスアームします。")
    vehicle.armed = False
    while vehicle.armed:
        time.sleep(1)
    print("ディスアームしました。")


def draw_banner(message_lines, term_title=None):
    """目立つバナーを描画する共通関数。

    ・ANSIエスケープで赤背景／黄背景・太字にして画面から浮き上がらせる
    ・上下を全角バーで囲み、大量のログに埋もれても視認できるようにする
    ・term_title を渡すと端末（タブ／ウィンドウ）のタイトルも書き換え、
      別ウィンドウ作業中でも気づけるようにする

    message_lines: (text, style) のリスト。style は "head"(赤) / "sub"(黄)。
    ANSIエスケープ非対応（パイプ出力など）の場合は装飾なしで表示する。
    """
    use_ansi = sys.stdout.isatty()
    bar = "█" * 56
    if use_ansi:
        head = "\033[1;97;41m"   # 太字・白文字・赤背景
        sub = "\033[1;30;103m"   # 太字・黒文字・黄背景
        rst = "\033[0m"
        if term_title is not None:
            sys.stdout.write("\033]0;%s\a" % term_title)
    else:
        head = sub = rst = ""

    styles = {"head": head, "sub": sub}
    lines = ["", "%s%s%s" % (head, bar, rst)]
    for text, style in message_lines:
        lines.append("%s%s%s" % (styles.get(style, head), text, rst))
    lines.append("%s%s%s" % (head, bar, rst))
    lines.append("")
    sys.stdout.write("\n".join(lines) + "\n")
    sys.stdout.flush()


def render_takeoff_prompt():
    """離陸確認プロンプトを目立つバナーで表示する。"""
    draw_banner(
        [("  ▶▶▶ 離陸してミッション（計測）を実行しますか？  ◀◀◀  ", "head"),
         ("      space または Enter = 実行 ／ 他の文字 + Enter = キャンセル  ", "sub")],
        term_title="!!! 離陸確認待ち !!!",
    )


def render_landing_prompt():
    """着陸確認プロンプトを目立つバナーで表示する。"""
    draw_banner(
        [("  ▶▶▶ 着陸を実行しますか？  ◀◀◀  ", "head"),
         ("      space または Enter = 着陸 ／ 他の文字 + Enter = 保留  ", "sub")],
        term_title="!!! 着陸確認待ち !!!",
    )


def update_wait_counter(elapsed_sec):
    """「応答待ち N 秒経過」を同じ行で更新表示する（改行しない）。

    行頭へ戻り（\\r）、行末までクリアしてから上書きするため、秒数が
    その場でカウントアップして見える。ANSI非対応時は装飾・クリアを省く。
    """
    use_ansi = sys.stdout.isatty()
    if use_ansi:
        sub = "\033[1;30;103m"
        rst = "\033[0m"
        clr = "\033[K"  # カーソル位置から行末までクリア
    else:
        sub = rst = clr = ""
    sys.stdout.write("\r%s%s      （応答待ち %d 秒経過）  %s" % (clr, sub, int(elapsed_sec), rst))
    sys.stdout.flush()


def render_low_battery_warning(reason):
    """バッテリー低下による強制RTL発動を目立つバナーで通知する（確認は不要）。"""
    draw_banner(
        [("  ⚠⚠⚠ バッテリー低下を検知 — 強制RTLします（確認不要） ⚠⚠⚠  ", "head"),
         ("      検知内容: %s  " % reason, "sub"),
         ("      RTL_ALTITUDE (%dm) まで上昇してホームへ帰還します  " % RTL_ALTITUDE, "sub")],
        term_title="!!! 強制RTL発動 !!!",
    )


def clear_terminal_title():
    """端末タイトルを既定へ戻す（確認待機の終了時に呼ぶ）。"""
    if sys.stdout.isatty():
        sys.stdout.write("\033]0;\a")
        sys.stdout.flush()


def save_csv(results):
    with open(OUTPUT_CSV, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f, fieldnames=[
                "leg", "elapsed_sec", "consumed_mah", "start_voltage", "end_voltage",
                "start_temp_c", "end_temp_c",
                "start_lat", "start_lon", "end_lat", "end_lon",
            ]
        )
        writer.writeheader()
        writer.writerows(results)


def main():
    print("Connecting to vehicle on: %s" % CONNECTION_STRING)
    vehicle = connect(CONNECTION_STRING, wait_ready=True)
    print("Connected.")

    # ---- バッテリー計測用の状態管理 ----
    state = {
        "measuring": False,
        "leg_number": 0,
        "start_time": None,
        "start_consumed_mah": None,
        "start_voltage": None,
        "last_pwm": None,
        "all_legs_done": False,
        "low_battery_rtl": False,
        "low_voltage_threshold": None,   # 起動時に自動判定する電圧下限[V]（Noneなら電圧保護は無効）
        "low_voltage_since": None,       # 電圧が下限を連続で下回り始めた時刻
    }
    battery = {"current_consumed": None, "voltage": None, "temp_c": None, "remaining": None}
    results = []

    def battery_status_listener(self, name, message):
        battery["current_consumed"] = message.current_consumed
        if message.voltages and message.voltages[0] not in (0, 65535):
            battery["voltage"] = message.voltages[0] / 1000.0
        else:
            battery["voltage"] = None
        # temperature: 単位はセンチ度(0.01degC)。INT16_MAX(32767)は未対応/不明を示す
        if message.temperature is not None and message.temperature != 32767:
            battery["temp_c"] = message.temperature / 100.0
        else:
            battery["temp_c"] = None
        # battery_remaining: 残量[%]。-1 は不明を示す
        if message.battery_remaining is not None and message.battery_remaining >= 0:
            battery["remaining"] = message.battery_remaining
        else:
            battery["remaining"] = None

        # ---- 全工程共通: 残量%または電圧下限を下回ったら強制RTL（ARM中のみ） ----
        if vehicle.armed and not state["low_battery_rtl"]:
            trigger_reason = None

            # (1) 主軸: 残量%ベース（セル数に依存しない）
            if battery["remaining"] is not None and battery["remaining"] < LOW_BATTERY_THRESHOLD:
                trigger_reason = "残量 %d%%（%d%%未満）" % (battery["remaining"], LOW_BATTERY_THRESHOLD)

            # (2) 保険: 電圧ベース（起動時に自動判定した下限）。負荷サグ誤発動を避けるため継続時間で判定
            elif state["low_voltage_threshold"] is not None and battery["voltage"] is not None:
                if battery["voltage"] < state["low_voltage_threshold"]:
                    if state["low_voltage_since"] is None:
                        state["low_voltage_since"] = time.time()
                    elif time.time() - state["low_voltage_since"] >= LOW_VOLTAGE_TIMER:
                        trigger_reason = "電圧 %.2fV（下限 %.2fV を %.0f秒継続で下回り）" % (
                            battery["voltage"], state["low_voltage_threshold"], LOW_VOLTAGE_TIMER)
                else:
                    # 下限を上回ったら継続タイマーをリセット
                    state["low_voltage_since"] = None

            if trigger_reason is not None:
                state["low_battery_rtl"] = True
                render_low_battery_warning(trigger_reason)
                vehicle.mode = VehicleMode("RTL")

    def servo_output_listener(self, name, message):
        attr_name = "servo%d_raw" % SERVO_CHANNEL
        pwm = getattr(message, attr_name, None)
        if pwm is None:
            return

        last_pwm = state["last_pwm"]
        state["last_pwm"] = pwm

        # 立ち上がりエッジ -> 区間「開始」
        if pwm >= PWM_THRESHOLD and (last_pwm is None or last_pwm < PWM_THRESHOLD) and not state["measuring"]:
            state["leg_number"] += 1
            state["measuring"] = True
            state["start_time"] = time.time()
            state["start_consumed_mah"] = battery["current_consumed"]
            state["start_voltage"] = battery["voltage"]
            state["start_temp_c"] = battery["temp_c"]
            loc = vehicle.location.global_frame
            state["start_lat"] = loc.lat
            state["start_lon"] = loc.lon

            print("[区間 %d/%d] 計測開始  起点消費電力=%s[mAh]" % (
                state["leg_number"], TOTAL_LEGS, state["start_consumed_mah"])) 

        # 立ち下がりエッジ -> 区間「終了」
        elif pwm < PWM_THRESHOLD and (last_pwm is None or last_pwm >= PWM_THRESHOLD) and state["measuring"]:
            state["measuring"] = False
            end_time = time.time()
            end_consumed_mah = battery["current_consumed"]
            end_voltage = battery["voltage"]
            end_temp_c = battery["temp_c"]
            end_loc = vehicle.location.global_frame
            end_lat = end_loc.lat
            end_lon = end_loc.lon

            elapsed = end_time - state["start_time"]

            consumed = None
            start_c = state["start_consumed_mah"]
            if start_c is not None and end_consumed_mah is not None \
                    and start_c >= 0 and end_consumed_mah >= 0:
                consumed = end_consumed_mah - start_c

            print("[区間 %d/%d] 計測終了  経過時間=%.1f[sec]  消費電力量=%s[mAh]  "
                  "電圧=%s[V] -> %s[V]" % (
                state["leg_number"], TOTAL_LEGS, elapsed, consumed,
                state["start_voltage"], end_voltage ))

            results.append({
                "leg": state["leg_number"],
                "elapsed_sec": round(elapsed, 1),
                "consumed_mah": consumed,
                "start_voltage": state["start_voltage"],
                "end_voltage": end_voltage,
                "start_lat": state["start_lat"],
                "start_lon": state["start_lon"],
                "end_lat": end_lat,
                "end_lon": end_lon,
                "start_temp_c": state["start_temp_c"],
                "end_temp_c": end_temp_c,
            })

            # 区間が終わるたびに都度保存（ホーム上空帰還時点でも確実にファイルに残る）
            save_csv(results)

            if state["leg_number"] >= TOTAL_LEGS:
                state["all_legs_done"] = True
                print("\n全区間（%d区間）の計測が完了しました。" % TOTAL_LEGS)
                print("計測結果を %s に保存しました。" % OUTPUT_CSV)

    vehicle.add_message_listener("BATTERY_STATUS", battery_status_listener)
    vehicle.add_message_listener("SERVO_OUTPUT_RAW", servo_output_listener)

    # ---- ARM前にバッテリーのセル数を手動入力（電圧ベース保護の下限[V]算出に使用） ----
    # 機体（セル数）が固定できないため、オペレーターが 1〜5 の数字でセル数を指定する。
    # spaceまたはEnter（空入力）はデフォルト 3S とする。
    while True:
        raw = input("バッテリーのセル数(S)を入力してください [1-5]（space/Enterで既定 %dS）: " % DEFAULT_CELL_COUNT)
        text = raw.strip()
        if text == "":
            cell_count = DEFAULT_CELL_COUNT
            break
        if text.isdigit() and 1 <= int(text) <= 5:
            cell_count = int(text)
            break
        print("1〜5 の数字、またはspace/Enter（既定 %dS）で入力してください。" % DEFAULT_CELL_COUNT)

    state["low_voltage_threshold"] = cell_count * LOW_CELL_VOLTAGE
    print("セル数=%dS で設定しました。電圧下限=%.2fV（%.2fV/セル×%dS）で保護します。" % (
        cell_count, state["low_voltage_threshold"], LOW_CELL_VOLTAGE, cell_count))

    # ---- ホーム座標をWayPointファイルNo.0行から読み取り（ARM前に検証しておく） ----
    # 緯度・経度のみ使用する。高度はARM後に機体の実測値を使う。
    home_lat, home_lon, _ = read_home_from_waypoints(WAYPOINT_FILE)
    print("WayPointファイルNo.0のホーム座標(緯度経度): lat=%.8f, lon=%.8f" % (
        home_lat, home_lon))

    # ---- 自動飛行シーケンス ----
    try:
        wait_until_armable(vehicle)

        # ---- ARM前のバッテリー残量チェック（20%未満なら離陸せず終了） ----
        if not check_battery_before_arm(battery):
            print("バッテリー残量不足のため、ARMせずに処理を終了します。CSVファイルは生成していません。")
            vehicle.close()
            return

        ensure_auto_options(vehicle)
        ensure_rtl_altitude(vehicle)

        # ---- ARM前にWayPointファイルを機体へ書き込む ----
        # Mission Planner等での手動書き込み忘れ／書き込み先の取り違えを防ぐため、
        # 本プログラムが毎回ファイルの内容を機体へ書き込む（既存ミッションは上書き）。
        upload_mission(vehicle, WAYPOINT_FILE)

        # ---- 書き込めたか検証（AUTO init failed の予防） ----
        # ミッションが無い状態で離陸すると、AUTO切替時に必ず失敗して10mでハマるため、
        # 離陸させる前にここで確認して中止する。
        verify_mission_loaded(vehicle)

        arm_vehicle(vehicle)

        # ---- ホームポイントをWayPointファイルNo.0の緯度経度に強制設定（高度は実測値） ----
        # ArduPilotはアーミング時に現在地をホームへ上書きするため、ARM後に設定する。
        set_home_location(vehicle, home_lat, home_lon)

        # ---- 離陸confirmationを求める ----
        # ARM完了後は1秒間隔でビープ音を鳴らしつつ y/n 入力を待つ。
        # 入力を待っている間にディスアーム状態に変化した場合は、その場で終了する。
        print("\nARMが完了しました（プロペラは低速回転中の可能性があります）。")
        # 見逃し防止: 目立つバナーで確認を促す（定期再表示は行わない）。
        render_takeoff_prompt()
        prompt_start = time.time()
        update_wait_counter(0.0)  # 経過秒はこの行を同じ位置で更新し続ける
        takeoff_confirmed = None
        last_beep = 0.0
        while takeoff_confirmed is None:
            # y/n 入力前にディスアーム状態へ変化したら終了
            if not vehicle.armed:
                clear_terminal_title()
                print("\nディスアーム状態を検知しました。処理を終了します。")
                vehicle.close()
                return

            # 低バッテリーで強制RTLが発動したら離陸せず終了
            if state["low_battery_rtl"]:
                clear_terminal_title()
                print("\nバッテリー低下により離陸を中止します。RTL/ディスアームを待機します。")
                while vehicle.armed:
                    time.sleep(1)
                print("ディスアームを確認しました。処理を終了します。")
                vehicle.close()
                return

            # 1秒間隔でビープ音（端末ベル）を鳴らす
            now = time.time()
            if now - last_beep >= 1.0:
                sys.stdout.write("\a")
                sys.stdout.flush()
                last_beep = now

            # 経過秒数を同じ位置で更新（改行しない）
            update_wait_counter(now - prompt_start)

            # 標準入力を非ブロッキングで確認（0.1秒待ち）
            # space または Enter（空入力）= 実行(yes)、それ以外 = キャンセル(no)
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if ready:
                answer = sys.stdin.readline().strip()
                takeoff_confirmed = (answer == "")

        clear_terminal_title()
        print("\n ")
        if not takeoff_confirmed:
            print("離陸を中止します。")
            disarm_vehicle(vehicle)
            print("ディスアームして終了しました。CSVファイルは生成していません。")
            vehicle.close()
            return

        # ---- AUTO開始前にGUIDEDで垂直離陸 ----
        # WayPoint先頭座標と現在地が離れていても、低高度での水平移動を避けるため、
        # 一旦真上へ TAKEOFF_ALTITUDE[m] 上昇してからミッションを開始する。
        guided_takeoff(vehicle, TAKEOFF_ALTITUDE)

        switch_to_auto(vehicle)

        print("ミッション実行中... 全区間の計測完了まで待機します。")
        print("(Ctrl+Cで途中終了した場合は、手動でRTL/LANDを実行してください)\n")

        while not state["all_legs_done"] and not state["low_battery_rtl"]:
            time.sleep(1)

        # ---- バッテリー低下による強制RTLの場合 ----
        if state["low_battery_rtl"]:
            print("バッテリー低下によりRTL中です。着陸・ディスアームを待機します。")
            while vehicle.armed:
                time.sleep(1)
            print("ディスアームを確認しました。RTL完了です。")
            save_csv(results)
            print("結果を %s に保存しました。" % OUTPUT_CSV)
            vehicle.close()
            return

        # ---- ホームポイント上空への到達を待機 ----
        print("\n機体はホームポイントへ移動します。")
        wait_until_home(vehicle, state)

        # ホーム到達待ちの間に低バッテリー強制RTLが発動した場合の処理
        if state["low_battery_rtl"]:
            print("バッテリー低下によりRTL中です。着陸・ディスアームを待機します。")
            while vehicle.armed:
                time.sleep(1)
            print("ディスアームを確認しました。RTL完了です。")
            save_csv(results)
            print("結果を %s に保存しました。" % OUTPUT_CSV)
            vehicle.close()
            return

        # ---- 着陸confirmationを求める ----
        # 見逃し防止: 目立つバナーで確認を促す（定期再表示は行わない）。
        prompt_start = time.time()
        render_landing_prompt()
        update_wait_counter(0.0)  # 経過秒はこの行を同じ位置で更新し続ける
        last_beep = 0.0
        while True:
            # 待機中に低バッテリー強制RTLが発動したら着陸確認を打ち切る
            if state["low_battery_rtl"]:
                clear_terminal_title()
                print("\nバッテリー低下によりRTL中です。着陸・ディスアームを待機します。")
                while vehicle.armed:
                    time.sleep(1)
                print("ディスアームを確認しました。RTL完了です。")
                save_csv(results)
                print("結果を %s に保存しました。" % OUTPUT_CSV)
                vehicle.close()
                return

            # 1秒間隔でビープ音（端末ベル）を鳴らす
            now = time.time()
            if now - last_beep >= 1.0:
                sys.stdout.write("\a")
                sys.stdout.flush()
                last_beep = now

            # 経過秒数を同じ位置で更新（改行しない）
            update_wait_counter(now - prompt_start)

            ready, _, _ = select.select([sys.stdin], [], [], 0.5)
            if not ready:
                continue
            # space または Enter（空入力）= 着陸(yes)、それ以外 = 保留(no)
            answer = sys.stdin.readline().strip()
            if answer == "":
                break
            else:
                print("\n着陸を保留しています。着陸するときは space または Enter を入力してください。")
                render_landing_prompt()
                update_wait_counter(now - prompt_start)

        clear_terminal_title()
        print("LANDモードに切り替えます。")
        vehicle.mode = VehicleMode("LAND")
        while vehicle.mode.name != "LAND":
            time.sleep(0.5)

        print("着陸中... ディスアームを待機します。")
        while vehicle.armed:
            time.sleep(1)
        print("ディスアームを確認しました。着陸完了です。")

    except TimeoutError as e:
        print("\nタイムアウトが発生しました: %s" % e)
        print("安全のためディスアームして終了します。")
        disarm_vehicle(vehicle)
        print("ディスアームして終了しました。CSVファイルは生成していません。")
        vehicle.close()
        return

    except RuntimeError as e:
        # AUTO切替失敗など、空中で発生しうるエラー。機体が飛行中の可能性があるため
        # ディスアームせず、安全にRTLで帰還・着陸させてから終了する。
        print("\nエラーが発生しました: %s" % e)
        if vehicle.armed:
            print("機体が飛行中の可能性があるため、安全のためRTLで帰還・着陸させます。")
            vehicle.mode = VehicleMode("RTL")
            while vehicle.armed:
                time.sleep(1)
            print("ディスアームを確認しました。RTL完了です。")
        save_csv(results)
        print("結果を %s に保存しました。" % OUTPUT_CSV)
        vehicle.close()
        return

    except KeyboardInterrupt:
        print("\n手動で中断されました。ここまでの結果を保存します。")

    # ---- 最終的な結果をCSVに保存（念のため再保存） ----
    save_csv(results)
    print("結果を %s に保存しました。" % OUTPUT_CSV)

    vehicle.close()


if __name__ == "__main__":
    main()