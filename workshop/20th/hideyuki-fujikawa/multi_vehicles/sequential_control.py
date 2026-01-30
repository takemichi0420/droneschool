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


def load_mission_from_file(filepath):
    """
    QGC WPL 110形式のミッションファイルを読み込む

    Args:
        filepath: ミッションファイルのパス

    Returns:
        list: ミッションアイテムのリスト
    """
    mission_items = []

    if not os.path.exists(filepath):
        print(f"警告: ミッションファイルが見つかりません: {filepath}")
        return mission_items

    with open(filepath, 'r') as f:
        lines = f.readlines()

    # 最初の行はヘッダー（QGC WPL 110）
    if not lines or not lines[0].startswith('QGC WPL'):
        print(f"警告: 無効なミッションファイル形式: {filepath}")
        return mission_items

    # 2行目以降がウェイポイント
    for line in lines[1:]:
        line = line.strip()
        if not line:
            continue

        parts = line.split('\t')
        if len(parts) < 12:
            continue

        # QGC WPL形式のパース
        # seq current frame command param1 param2 param3 param4 x y z autocontinue
        seq = int(parts[0])
        current = int(parts[1])
        frame = int(parts[2])
        command = int(parts[3])
        param1 = float(parts[4])
        param2 = float(parts[5])
        param3 = float(parts[6])
        param4 = float(parts[7])
        x = int(float(parts[8]) * 1e7)  # 緯度を整数に変換
        y = int(float(parts[9]) * 1e7)  # 経度を整数に変換
        z = float(parts[10])
        autocontinue = int(parts[11])

        mission_items.append({
            'seq': seq,
            'current': current,
            'frame': frame,
            'command': command,
            'param1': param1,
            'param2': param2,
            'param3': param3,
            'param4': param4,
            'x': x,
            'y': y,
            'z': z,
            'autocontinue': autocontinue
        })

    return mission_items


class VehicleController:
    """機体制御クラス"""

    def __init__(self, name: str, connection_string: str, vehicle_type: str, mission_file: str = None):
        """
        Args:
            name: 機体名（表示用）
            connection_string: 接続文字列（例: "tcp:127.0.0.1:5762"）
            vehicle_type: 機体タイプ（"rover", "boat", "copter"）
            mission_file: ミッションファイルのパス（オプション）
        """
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
        print(f"[{self.name}] 接続完了 (sysid: {self.master.target_system}, compid: {self.master.target_component})")

    def arm(self):
        """機体をアーム"""
        print(f"[{self.name}] アーム開始...")

        if self.vehicle_type == "copter":
            # コプターの場合はGUIDEDモードに変更
            mode = 'GUIDED'
            self.master.set_mode_apm(self.master.mode_mapping()[mode])

            # モード変更を確認
            while True:
                if self.master.flightmode == mode:
                    break
                self.master.recv_msg()
            print(f"[{self.name}] モード変更完了: {mode}")

            self.master.arducopter_arm()
        else:
            # ローバー、ボートの場合はGUIDEDモードに変更してからアーム
            mode = 'GUIDED'
            self.master.set_mode_apm(self.master.mode_mapping()[mode])

            # モード変更を確認
            while True:
                if self.master.flightmode == mode:
                    break
                self.master.recv_msg()
            print(f"[{self.name}] モード変更完了: {mode}")

            self.master.arducopter_arm()

        self.master.motors_armed_wait()
        print(f"[{self.name}] アーム完了")

    def takeoff(self, target_altitude: float = 3.0):
        """
        離陸（コプターのみ）

        Args:
            target_altitude: 目標高度（メートル）
        """
        if self.vehicle_type != "copter":
            print(f"[{self.name}] 離陸スキップ（コプター以外）")
            return

        print(f"[{self.name}] 離陸開始（目標高度: {target_altitude}m）...")

        # 離陸コマンド送信
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, target_altitude)

        # メッセージレート変更: GLOBAL_POSITION_INT(33)を10Hzで受信
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, 33, 100000, 0, 0, 0, 0, 0)

        # 目標高度への到達を確認
        while True:
            received_msg = self.master.recv_match(
                type='GLOBAL_POSITION_INT', blocking=True)
            current_altitude = received_msg.relative_alt / 1000

            print(f"[{self.name}] 高度: {current_altitude:.2f}m")

            if current_altitude >= target_altitude * 0.95:
                print(f"[{self.name}] 目標高度に到達")
                break

            time.sleep(0.1)

    def start_mission(self):
        """ミッション開始"""
        print(f"[{self.name}] ミッション開始...")

        # AUTOモードに変更
        mode = 'AUTO'
        self.master.set_mode_apm(self.master.mode_mapping()[mode])

        # モード変更を確認
        while True:
            if self.master.flightmode == mode:
                break
            self.master.recv_msg()

        print(f"[{self.name}] モード変更完了: {mode}")

    def upload_mission(self, mission_items):
        """
        ミッションをアップロード

        Args:
            mission_items: ミッションアイテムのリスト
        """
        if not mission_items:
            print(f"[{self.name}] ミッションアイテムがありません")
            return

        print(f"[{self.name}] ミッションアップロード開始（{len(mission_items)}個のウェイポイント）")

        # ミッションをクリア
        self.master.mav.mission_clear_all_send(
            self.master.target_system, self.master.target_component)
        time.sleep(0.5)

        # ミッション数を送信
        self.master.mav.mission_count_send(
            self.master.target_system, self.master.target_component, len(mission_items))

        # 各ミッションアイテムを送信
        for item in mission_items:
            # MISSION_REQUEST または MISSION_REQUEST_INT を待機
            ack = self.master.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT'], blocking=True, timeout=5)
            if not ack:
                print(f"[{self.name}] タイムアウト: ミッションリクエストを受信できませんでした")
                return

            # ミッションアイテムを送信
            self.master.mav.mission_item_int_send(
                self.master.target_system,
                self.master.target_component,
                item['seq'],
                item['frame'],
                item['command'],
                item['current'],
                item['autocontinue'],
                item['param1'],
                item['param2'],
                item['param3'],
                item['param4'],
                item['x'],
                item['y'],
                item['z'],
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION
            )

        # アップロード完了を待機
        ack = self.master.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
        if ack and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
            print(f"[{self.name}] ミッションアップロード完了")
        else:
            print(f"[{self.name}] ミッションアップロード失敗")

    def get_mission_count(self):
        """ミッションアイテム数を取得"""
        self.master.mav.mission_request_list_send(
            self.master.target_system, self.master.target_component)
        mission_count_msg = self.master.recv_match(
            type='MISSION_COUNT', blocking=True, timeout=5)
        if mission_count_msg:
            return mission_count_msg.count
        return 0

    def get_distance_to_waypoint(self, target_lat, target_lon):
        """
        現在位置から目標ウェイポイントまでの距離を計算（メートル）

        Args:
            target_lat: 目標緯度（度）
            target_lon: 目標経度（度）

        Returns:
            float: 距離（メートル）
        """
        # GLOBAL_POSITION_INTメッセージを取得
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
        if not msg:
            return float('inf')

        # 現在位置（度に変換）
        current_lat = msg.lat / 1e7
        current_lon = msg.lon / 1e7

        # Haversine公式で距離計算
        R = 6371000  # 地球の半径（メートル）

        lat1 = math.radians(current_lat)
        lat2 = math.radians(target_lat)
        dlat = math.radians(target_lat - current_lat)
        dlon = math.radians(target_lon - current_lon)

        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

        distance = R * c
        return distance

    def wait_mission_complete(self):
        """ミッション完了を待機（位置情報ベース）"""
        print(f"[{self.name}] ミッション完了待機中...")

        # ミッションをダウンロードして最後のウェイポイント情報を取得
        mission_count = self.get_mission_count()
        print(f"[{self.name}] ミッションアイテム数: {mission_count}")

        if mission_count == 0:
            print(f"[{self.name}] 警告: ミッションがアップロードされていません")
            return

        # 最後のウェイポイント番号（HOMEを除く）
        last_waypoint_seq = mission_count - 1

        # 最後のウェイポイントの座標を取得
        self.master.mav.mission_request_int_send(
            self.master.target_system, self.master.target_component, last_waypoint_seq)
        last_wp = self.master.recv_match(type='MISSION_ITEM_INT', blocking=True, timeout=5)

        if not last_wp:
            print(f"[{self.name}] 警告: 最後のウェイポイント情報を取得できませんでした")
            return

        target_lat = last_wp.x / 1e7
        target_lon = last_wp.y / 1e7
        print(f"[{self.name}] 最後のウェイポイント座標: ({target_lat:.6f}, {target_lon:.6f})")

        # GLOBAL_POSITION_INTメッセージのレート設定（10Hz）
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, 33, 100000, 0, 0, 0, 0, 0)  # GLOBAL_POSITION_INT = 33

        time.sleep(1)  # メッセージレート設定の反映を待つ

        last_distance = float('inf')
        reached_threshold = 5.0  # 到達判定距離（メートル）
        within_threshold_count = 0  # 閾値内にいる回数
        required_count = 5  # 完了とみなすために必要な連続カウント

        print(f"[{self.name}] 最後のウェイポイント({last_waypoint_seq})への到達を待機中（距離ベース）...")

        while True:
            # GLOBAL_POSITION_INTメッセージを取得
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)

            if msg:
                current_lat = msg.lat / 1e7
                current_lon = msg.lon / 1e7

                # 距離計算（Haversine公式）
                R = 6371000  # 地球の半径（メートル）
                lat1 = math.radians(current_lat)
                lat2 = math.radians(target_lat)
                dlat = math.radians(target_lat - current_lat)
                dlon = math.radians(target_lon - current_lon)

                a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
                c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
                distance = R * c

                # 距離が大きく変わったらログ出力
                if abs(distance - last_distance) > 5:
                    print(f"[{self.name}] 最終WPまで: {distance:.1f}m")
                    last_distance = distance

                # 距離ベースの到達判定
                if distance < reached_threshold:
                    within_threshold_count += 1
                    if within_threshold_count >= required_count:
                        print(f"[{self.name}] ★★★ 位置判定: 最終WP({last_waypoint_seq})に到達 ({distance:.1f}m) ★★★")
                        time.sleep(2)
                        self.master.recv_msg()
                        current_mode = self.master.flightmode
                        print(f"[{self.name}] ミッション完了（最終モード: {current_mode}）")
                        return
                    else:
                        print(f"[{self.name}] 最終WPに接近中: {distance:.1f}m (確認: {within_threshold_count}/{required_count})")
                else:
                    # 閾値外に出たらカウントリセット
                    within_threshold_count = 0

            time.sleep(0.1)

    def close(self):
        """接続を閉じる"""
        if self.master:
            print(f"[{self.name}] 接続を閉じます")
            self.master.close()


def main():
    """メイン処理"""

    print("=" * 60)
    print("複数機体順次制御スクリプト")
    print("ローバー → ボート → コプター")
    print("=" * 60)

    # 接続先ホストの設定（環境変数またはデフォルト）
    # 環境変数 SITL_HOST を設定することで変更可能
    # 例: export SITL_HOST=192.168.1.100
    sitl_host = os.environ.get('SITL_HOST', '127.0.0.1')
    print(f"接続先ホスト: {sitl_host}")

    # スクリプトのディレクトリを取得
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # 機体定義（ミッションファイル付き）
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
            print(f"\n{'=' * 60}")
            print(f"[{vehicle.name}] 制御開始")
            print(f"{'=' * 60}")

            # 接続
            vehicle.connect()
            time.sleep(1)

            # ミッションファイルがある場合はアップロード
            if vehicle.mission_file:
                mission_items = load_mission_from_file(vehicle.mission_file)
                if mission_items:
                    vehicle.upload_mission(mission_items)
                    time.sleep(1)
                else:
                    print(f"[{vehicle.name}] 警告: ミッションファイルの読み込みに失敗しました")

            # アーム
            vehicle.arm()
            time.sleep(1)

            # 離陸（コプターのみ）
            if vehicle.vehicle_type == "copter":
                vehicle.takeoff(target_altitude=3.0)
                time.sleep(1)

            # ミッション開始
            vehicle.start_mission()

            # ミッション完了待機
            vehicle.wait_mission_complete()

            print(f"\n[{vehicle.name}] 制御完了")

            # 次の機体に進む前に少し待機
            time.sleep(2)

        print("\n" + "=" * 60)
        print("全機体の制御が完了しました")
        print("=" * 60)

    except KeyboardInterrupt:
        print("\n\nユーザーによる中断")

    except Exception as e:
        print(f"\nエラーが発生しました: {e}")
        import traceback
        traceback.print_exc()

    finally:
        # 全機体の接続を閉じる
        print("\n接続をクローズ中...")
        for vehicle in vehicles:
            vehicle.close()


if __name__ == '__main__':
    main()
