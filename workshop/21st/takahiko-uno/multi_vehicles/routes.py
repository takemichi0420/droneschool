# -*- coding: utf-8 -*-
"""
配送ルート定義とミッション自動生成（ローバー / ボート / コプター）

課題で指定された座標:
  荷物の起点（滑川駅）    : 35.876991, 140.348026
  対岸ポート              : 35.879768, 140.348495   … 利根川 右岸(南東岸)
  メインポート            : 35.878275, 140.338069   … 利根川 左岸(北西岸)
  配送先（セブンイレブン）: 35.877518, 140.295439

ルート条件:
  - ローバーは可能な範囲で一般道路を通る
  - ボートは河川（利根川）上を通る
  - 指定座標以外の中間点は自由

中間点の出どころ（すべて衛星写真（地理院タイル）に重ねて位置を確認済み）:
  - ローバー: OpenStreetMap の道路網から作った経路グラフ上の最短経路(約380m)。
    駅前 → 成田線の踏切 → 堤防の道路（下総神崎バイパス）へ北西に直進 → 船着き場。
    ※OSRM の車向け経路探索では南西へ約250m迂回してバイパスに合流する経路(719m)になる。
      堤防への取り付け（下記 35.878618→35.878871 の約31m）が OSM 上は歩行者用の坂道として
      登録されているため、車の経路探索では使われない。配送ローバーは通行できる幅なので採用した。
  - ボート  : OpenStreetMap の 利根川(way 60623787/60624705/60617499)の
    河川中心線上の点と、各ポートの沖合130mの離着岸点。全点が水面上。
  - コプター: メインポート→セブンイレブンの直線上（水田地帯の上空）を等分した点。

このモジュールは上記の座標から MAVLink のミッション(MISSION_ITEM_INT)を生成し、
機体へアップロードする。Mission Planner で開ける .waypoints 形式での書き出しにも対応。
"""

from pymavlink import mavutil

# ==== 指定座標 ====
NAMEGAWA_STATION = (35.876991, 140.348026)   # 荷物の起点（滑川駅）
OPPOSITE_PORT = (35.879768, 140.348495)      # 対岸ポート（利根川 右岸）
MAIN_PORT = (35.878275, 140.338069)          # メインポート（利根川 左岸）
SEVEN_ELEVEN = (35.877518, 140.295439)       # 配送先（セブンイレブン）


class Route:
    """1レグのルート定義。"""

    def __init__(self, key, origin_name, destination_name, waypoints,
                 cruise_speed=None, takeoff_alt=None, cruise_alt=None, land_at_goal=False,
                 lean_angle_max=None):
        self.key = key                            # "rover" / "boat" / "copter"
        self.origin_name = origin_name
        self.destination_name = destination_name
        self.waypoints = waypoints                # [(lat, lon), ...] 先頭=出発地, 末尾=目的地
        self.cruise_speed = cruise_speed          # 巡航速度[m/s]（DO_CHANGE_SPEEDで設定）
        self.takeoff_alt = takeoff_alt            # 離陸高度[m]（コプターのみ）
        self.cruise_alt = cruise_alt              # 巡航高度[m]（コプターのみ）
        self.land_at_goal = land_at_goal          # 目的地で着陸(NAV_LAND)するか
        self.lean_angle_max = lean_angle_max      # 最大傾斜角[度]（コプターのみ・速度の上限を決める）

    @property
    def start(self):
        return self.waypoints[0]

    @property
    def goal(self):
        return self.waypoints[-1]


# ---------------------------------------------------------------------------
# ルート1: ローバー（滑川駅 → 対岸ポート）約 380m
#   駅前から北東へ進んで成田線の踏切を渡り、そこから堤防の道路（下総神崎バイパス）まで
#   北西へ直進、堤防の道路に出てから船着き場へ下る。
# ---------------------------------------------------------------------------
ROVER_ROUTE = Route(
    key="rover",
    origin_name="滑川駅",
    destination_name="対岸ポート",
    cruise_speed=8.0,
    waypoints=[
        NAMEGAWA_STATION,          # 滑川駅（荷物の起点）
        (35.877103, 140.348242),   # 駅前から県道207号（滑河操車場線）へ
        (35.877280, 140.348297),   # 北東へ
        (35.877924, 140.348877),   # 成田線の踏切を渡り、交差点で左折
        (35.878123, 140.348653),   # 堤防に向かって北西へ直進
        (35.878409, 140.348296),   #   〃
        (35.878618, 140.348068),   # 堤防下の道に突き当たる
        (35.878871, 140.348207),   # 堤防の道路（下総神崎バイパス）へ取り付く
        (35.879010, 140.348240),   # 川側へ下る道へ
        (35.879355, 140.348499),
        (35.879700, 140.348433),   # 船着き場の手前
        OPPOSITE_PORT,             # 対岸ポート（右岸の船着き場）
    ],
)

# ---------------------------------------------------------------------------
# ルート2: ボート（対岸ポート → メインポート）約 1130m
#   右岸を離岸して利根川の中心線に出て、川を遡上しながら左岸のメインポートへ渡る。
# ---------------------------------------------------------------------------
BOAT_ROUTE = Route(
    key="boat",
    origin_name="対岸ポート",
    destination_name="メインポート",
    cruise_speed=8.0,
    waypoints=[
        OPPOSITE_PORT,             # 対岸ポート（右岸）
        (35.880676, 140.347588),   # 離岸（沖へ約130m、航路へ出る）
        (35.879523, 140.343536),   # 利根川 中心線上
        (35.878002, 140.341329),   # 利根川 中心線上
        (35.877367, 140.338976),   # メインポート沖 約130m（着岸へ向きを変える）
        MAIN_PORT,                 # メインポート（左岸）
    ],
)

# ---------------------------------------------------------------------------
# ルート3: コプター（メインポート → セブンイレブン）約 3.9km
#   水田地帯の上空を高度 cruise_alt[m] で西進し、配送先に着陸する。
#
#   コプターの水平速度は「機体をどれだけ傾けられるか」で決まる（傾き↔空気抵抗の釣り合い）。
#   既定の最大傾斜角 30度では、巡航速度をいくら大きく指定しても約10m/sで頭打ちになるため、
#   lean_angle_max で傾斜角の上限も上げる（45度で約14.4m/s。SITLでの実測値）。
#   20m/s を出すには約66度の傾斜が必要で現実的でないため、指定速度には届かない。
# ---------------------------------------------------------------------------
COPTER_ROUTE = Route(
    key="copter",
    origin_name="メインポート",
    destination_name="セブンイレブン",
    cruise_speed=20.0,
    lean_angle_max=45.0,
    takeoff_alt=40.0,
    cruise_alt=40.0,
    land_at_goal=True,
    waypoints=[
        MAIN_PORT,                 # メインポート（離陸地点）
        (35.878023, 140.323859),   # 水田地帯の上空（区間 1/3）
        (35.877770, 140.309649),   # 水田地帯の上空（区間 2/3）
        SEVEN_ELEVEN,              # セブンイレブン（配送先・着陸地点）
    ],
)

ROUTES = {
    ROVER_ROUTE.key: ROVER_ROUTE,
    BOAT_ROUTE.key: BOAT_ROUTE,
    COPTER_ROUTE.key: COPTER_ROUTE,
}


def reversed_route(route):
    """ルートを逆向きにした Route を返す（配送先から出発地へ戻す回送用）。

    ウェイポイントの順序と出発地／目的地の名前を入れ替えるだけで、
    速度・高度・着陸の有無は配送時と同じ設定を使う。
    ローバーは同じ一般道を逆走し、ボートは同じ航路を下り、
    コプターは配送先から離陸して元の出発地に着陸する。
    """
    return Route(
        key=route.key,
        origin_name=route.destination_name,
        destination_name=route.origin_name,
        waypoints=list(reversed(route.waypoints)),
        cruise_speed=route.cruise_speed,
        takeoff_alt=route.takeoff_alt,
        cruise_alt=route.cruise_alt,
        land_at_goal=route.land_at_goal,
        lean_angle_max=route.lean_angle_max,
    )


# ---------------------------------------------------------------------------
# ミッション生成
# ---------------------------------------------------------------------------

def build_mission(route, target_system=0, target_component=0):
    """ルート定義から MISSION_ITEM_INT のリスト（=ミッション）を生成する。

    構成:
      seq=0                : ホーム（出発地。ArduPilotはアーム時の現在地で上書きする）
      NAV_TAKEOFF          : コプターのみ（takeoff_alt まで離陸）
      DO_CHANGE_SPEED      : 巡航速度の指定（cruise_speed がある場合）
      NAV_WAYPOINT ...     : 中間点〜目的地
      NAV_LAND             : land_at_goal の場合、目的地で着陸
    """
    items = []

    def add(command, lat, lon, alt, frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            param1=0.0, param2=0.0, param3=0.0, param4=0.0):
        items.append(mavutil.mavlink.MAVLink_mission_item_int_message(
            target_system, target_component,
            len(items),          # seq
            frame, command,
            0,                   # current
            1,                   # autocontinue
            param1, param2, param3, param4,
            int(round(lat * 1e7)), int(round(lon * 1e7)), float(alt)))

    # seq=0: ホーム。frame は絶対高度、高度は0でよい（機体側が実測値で上書きする）
    add(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, route.start[0], route.start[1], 0.0,
        frame=mavutil.mavlink.MAV_FRAME_GLOBAL)

    # 離陸（コプターのみ）
    if route.takeoff_alt:
        add(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0.0, 0.0, route.takeoff_alt)

    # 巡航速度の指定（param1: 1=対地速度, param2: 速度[m/s], param3: スロットル変更なし)
    if route.cruise_speed:
        add(mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0.0, 0.0, 0.0,
            frame=mavutil.mavlink.MAV_FRAME_MISSION,
            param1=1.0, param2=route.cruise_speed, param3=-1.0)

    # 中間点〜目的地（seq=0 の出発地は機体が既にいる場所なので入れない）
    alt = route.cruise_alt if route.cruise_alt else 0.0
    intermediate = route.waypoints[1:-1]
    for lat, lon in intermediate:
        add(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, lat, lon, alt)

    goal_lat, goal_lon = route.goal
    if route.land_at_goal:
        # 配送先の上空まで飛んでから着陸する
        add(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, goal_lat, goal_lon, alt)
        add(mavutil.mavlink.MAV_CMD_NAV_LAND, goal_lat, goal_lon, 0.0)
    else:
        add(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, goal_lat, goal_lon, alt)

    return items


def upload_mission(master, items, timeout=30.0, retries=4):
    """生成したミッションを機体へアップロードする（MAVLink ミッションプロトコル）。

    MISSION_COUNT を送ると、機体が MISSION_REQUEST(_INT) で1件ずつ要求してくるので、
    その seq のアイテムを送り返す。最後に MISSION_ACK が返る。

    起動直後の機体は、ミッション保存領域の準備が終わっておらず
    MAV_MISSION_NO_SPACE 等で一旦拒否してくることがあるため、拒否・タイムアウトの
    どちらの場合も少し待って再試行する。
    """
    import time

    last_error = None
    for attempt in range(1, retries + 1):
        # 送信するアイテムの宛先を、実際の接続先に合わせる
        for item in items:
            item.target_system = master.target_system
            item.target_component = master.target_component

        master.mav.mission_count_send(
            master.target_system, master.target_component, len(items))

        deadline = time.time() + timeout
        sent = set()
        rejected = None
        while time.time() < deadline:
            msg = master.recv_match(
                type=["MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"],
                blocking=True, timeout=2)
            if msg is None:
                continue
            if msg.get_type() == "MISSION_ACK":
                if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    return len(items)
                rejected = mavutil.mavlink.enums["MAV_MISSION_RESULT"][msg.type].name
                break
            if 0 <= msg.seq < len(items):
                master.mav.send(items[msg.seq])
                sent.add(msg.seq)

        if rejected is not None:
            last_error = "機体がアップロードを拒否しました（%s）" % rejected
        else:
            last_error = "応答待ちがタイムアウトしました（送信済み %d/%d）" % (len(sent), len(items))

        if attempt < retries:
            print("  - ミッションのアップロードに失敗: %s。%d秒待って再試行します（%d/%d）。"
                  % (last_error, 3, attempt, retries))
            time.sleep(3.0)

    raise RuntimeError("ミッションのアップロードが完了しませんでした: %s" % last_error)


def export_waypoints(route, path):
    """Mission Planner で開ける QGC WPL 110 形式(.waypoints)で書き出す。

    プログラムが生成したミッションを目で確認したり、手動で機体へ書き込む場合に使う。
    """
    items = build_mission(route)
    lines = ["QGC WPL 110"]
    for item in items:
        lines.append("\t".join([
            str(item.seq),
            "1" if item.seq == 0 else "0",     # current
            str(item.frame),
            str(item.command),
            "%.8f" % item.param1, "%.8f" % item.param2,
            "%.8f" % item.param3, "%.8f" % item.param4,
            "%.8f" % (item.x / 1e7), "%.8f" % (item.y / 1e7), "%.6f" % item.z,
            str(item.autocontinue),
        ]))
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    return path


def route_length_m(route):
    """ルートの総距離[m]（水平距離の近似）。"""
    import math

    total = 0.0
    for i in range(len(route.waypoints) - 1):
        (lat1, lon1), (lat2, lon2) = route.waypoints[i], route.waypoints[i + 1]
        dlat = lat2 - lat1
        dlon = (lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2.0))
        total += math.sqrt(dlat * dlat + dlon * dlon) * 1.113195e5
    return total
