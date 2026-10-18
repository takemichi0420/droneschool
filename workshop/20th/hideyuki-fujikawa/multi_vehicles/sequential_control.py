#!/usr/bin/env python3
"""
複数機体順次制御スクリプト

ローバー → ボート → コプター の順で制御を実行します。
前の機体がミッション完了したら、次の機体がスタートします。

各機体の動作:
- ローバー、ボート: アーム → ミッション開始 → ミッション完了待機
- コプター: アーム → 離陸 → ミッション開始 → ミッション完了待機

前提条件:
- 各機体にミッションが事前にアップロードされていること
- SITL が起動していること（multi_vehicle.bat または multi_vehicle_dialog.bat で起動）
"""

from pymavlink import mavutil
import time
import os
import math

# 定数定義
MESSAGE_RATE_HZ = 10  # メッセージ受信レート（Hz）
MESSAGE_INTERVAL_US = 100000  # メッセージ間隔（マイクロ秒）
WAYPOINT_REACHED_THRESHOLD_M = 5.0  # ウェイポイント到達判定距離（メートル）
REQUIRED_CONFIRMATION_COUNT = 5  # 到達確認に必要な連続カウント
EARTH_RADIUS_M = 6371000  # 地球の半径（メートル）
DEFAULT_TAKEOFF_ALTITUDE_M = 3.0  # デフォルト離陸高度（メートル）


def load_mission_from_file(filepath):
    """QGC WPL 110形式のミッションファイルを読み込む"""
    if not os.path.exists(filepath):
        print(f"警告: ミッションファイルが見つかりません: {filepath}")
        return []

    with open(filepath, 'r') as f:
        lines = f.readlines()

    if not lines or not lines[0].startswith('QGC WPL'):
        print(f"警告: 無効なミッションファイル形式: {filepath}")
        return []

    mission_items = []
    for line in lines[1:]:
        parts = line.strip().split('\t')
        if len(parts) < 12:
            continue

        mission_items.append({
            'seq': int(parts[0]),
            'current': int(parts[1]),
            'frame': int(parts[2]),
            'command': int(parts[3]),
            'param1': float(parts[4]),
            'param2': float(parts[5]),
            'param3': float(parts[6]),
            'param4': float(parts[7]),
            'x': int(float(parts[8]) * 1e7),
            'y': int(float(parts[9]) * 1e7),
            'z': float(parts[10]),
            'autocontinue': int(parts[11])
        })

    return mission_items


class VehicleController:
    """機体制御クラス"""

    def __init__(self, name: str, connection_string: str, vehicle_type: str, mission_file: str = None):
        self.name = name
        self.connection_string = connection_string
        self.vehicle_type = vehicle_type
        self.mission_file = mission_file
        self.master = None

    def connect(self):
        """機体に接続"""
        print(f"\n[{self.name}] 接続中: {self.connection_string}")
        self.master = mavutil.mavlink_connection(
            self.connection_string, source_system=1, source_component=90)
        self.master.wait_heartbeat()
        print(f"[{self.name}] 接続完了")

    def _change_mode(self, mode: str):
        """フライトモードを変更"""
        self.master.set_mode_apm(self.master.mode_mapping()[mode])
        while self.master.flightmode != mode:
            self.master.recv_msg()
        print(f"[{self.name}] {mode}モードに変更")

    def arm(self):
        """機体をアーム"""
        self._change_mode('GUIDED')
        self.master.arducopter_arm()
        self.master.motors_armed_wait()
        print(f"[{self.name}] アーム完了")

    def _set_message_interval(self, message_id: int):
        """メッセージ受信間隔を設定"""
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, message_id, MESSAGE_INTERVAL_US, 0, 0, 0, 0, 0)

    def takeoff(self, target_altitude: float = DEFAULT_TAKEOFF_ALTITUDE_M):
        """離陸（コプターのみ）"""
        if self.vehicle_type != "copter":
            return

        print(f"[{self.name}] 離陸開始（目標: {target_altitude}m）")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, target_altitude)
        self._set_message_interval(33)

        while True:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            alt = msg.relative_alt / 1000
            if alt >= target_altitude * 0.95:
                print(f"[{self.name}] 目標高度到達")
                break
            time.sleep(0.1)

    def start_mission(self):
        """ミッション開始"""
        self._change_mode('AUTO')

    def upload_mission(self, mission_items):
        """ミッションをアップロード"""
        if not mission_items:
            return

        print(f"[{self.name}] ミッションアップロード開始（{len(mission_items)}個）")
        self.master.mav.mission_clear_all_send(
            self.master.target_system, self.master.target_component)
        time.sleep(0.5)

        self.master.mav.mission_count_send(
            self.master.target_system, self.master.target_component, len(mission_items))

        for item in mission_items:
            if not self.master.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT'],
                                          blocking=True, timeout=5):
                print(f"[{self.name}] ミッションアップロードタイムアウト")
                return

            self.master.mav.mission_item_int_send(
                self.master.target_system, self.master.target_component,
                item['seq'], item['frame'], item['command'], item['current'],
                item['autocontinue'], item['param1'], item['param2'],
                item['param3'], item['param4'], item['x'], item['y'], item['z'],
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

        ack = self.master.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
        status = "完了" if ack and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED else "失敗"
        print(f"[{self.name}] ミッションアップロード{status}")

    def get_mission_count(self):
        """ミッションアイテム数を取得"""
        self.master.mav.mission_request_list_send(
            self.master.target_system, self.master.target_component)
        mission_count_msg = self.master.recv_match(
            type='MISSION_COUNT', blocking=True, timeout=5)
        if mission_count_msg:
            return mission_count_msg.count
        return 0

    def _calculate_distance(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Haversine公式で2地点間の距離を計算（メートル）"""
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return EARTH_RADIUS_M * c

    def get_distance_to_waypoint(self, target_lat: float, target_lon: float) -> float:
        """現在位置から目標ウェイポイントまでの距離を計算"""
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
        if not msg:
            return float('inf')
        return self._calculate_distance(msg.lat / 1e7, msg.lon / 1e7, target_lat, target_lon)

    def _get_last_waypoint_position(self):
        """最後のウェイポイントの座標を取得"""
        mission_count = self.get_mission_count()
        if mission_count == 0:
            print(f"[{self.name}] 警告: ミッションが未アップロード")
            return None, None, None

        last_seq = mission_count - 1
        self.master.mav.mission_request_int_send(
            self.master.target_system, self.master.target_component, last_seq)
        wp = self.master.recv_match(type='MISSION_ITEM_INT', blocking=True, timeout=5)

        if not wp:
            print(f"[{self.name}] 警告: ウェイポイント情報取得失敗")
            return None, None, None

        return wp.x / 1e7, wp.y / 1e7, last_seq

    def wait_mission_complete(self):
        """ミッション完了を待機"""
        target_lat, target_lon, last_seq = self._get_last_waypoint_position()
        if target_lat is None:
            return

        self._set_message_interval(33)
        time.sleep(1)

        last_dist = float('inf')
        count = 0

        while True:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if not msg:
                continue

            # SIM_SPEEDUPで実時間あたりのメッセージ生成レートが上がるため、
            # バッファに溜まった分を読み捨てて常に最新の位置情報を使う
            while True:
                newer = self.master.recv_match(
                    type='GLOBAL_POSITION_INT', blocking=False)
                if newer is None:
                    break
                msg = newer

            dist = self._calculate_distance(msg.lat / 1e7, msg.lon / 1e7, target_lat, target_lon)

            if abs(dist - last_dist) > 5:
                print(f"[{self.name}] 最終WPまで: {dist:.1f}m")
                last_dist = dist

            if dist < WAYPOINT_REACHED_THRESHOLD_M:
                count += 1
                if count >= REQUIRED_CONFIRMATION_COUNT:
                    print(f"[{self.name}] ミッション完了（WP{last_seq}到達）")
                    return
            else:
                count = 0

    def close(self):
        """接続を閉じる"""
        if self.master:
            self.master.close()


def main():
    """メイン処理"""
    print("=" * 60)
    print("複数機体順次制御: ローバー → ボート → コプター")
    print("=" * 60)

    sitl_host = os.environ.get('SITL_HOST', '192.168.3.38')
    script_dir = os.path.dirname(os.path.abspath(__file__))

    vehicles = [
        VehicleController("ローバー", f"tcp:{sitl_host}:5762", "rover",
                         os.path.join(script_dir, "rover_mission.waypoints")),
        VehicleController("ボート", f"tcp:{sitl_host}:5772", "boat",
                         os.path.join(script_dir, "boat_mission.waypoints")),
        VehicleController("コプター", f"tcp:{sitl_host}:5782", "copter",
                         os.path.join(script_dir, "copter_mission.waypoints")),
    ]

    try:
        for vehicle in vehicles:
            print(f"\n{'=' * 60}\n[{vehicle.name}] 制御開始\n{'=' * 60}")

            vehicle.connect()
            time.sleep(1)

            if vehicle.mission_file:
                mission_items = load_mission_from_file(vehicle.mission_file)
                if mission_items:
                    vehicle.upload_mission(mission_items)
                    time.sleep(1)

            vehicle.arm()
            time.sleep(1)

            if vehicle.vehicle_type == "copter":
                vehicle.takeoff()
                time.sleep(1)

            vehicle.start_mission()
            vehicle.wait_mission_complete()

            print(f"\n[{vehicle.name}] 制御完了")
            time.sleep(2)

        print("\n" + "=" * 60 + "\n全機体の制御が完了しました\n" + "=" * 60)

    except KeyboardInterrupt:
        print("\n中断されました")
    except Exception as e:
        print(f"\nエラー: {e}")
        import traceback
        traceback.print_exc()
    finally:
        for vehicle in vehicles:
            vehicle.close()


if __name__ == '__main__':
    main()
