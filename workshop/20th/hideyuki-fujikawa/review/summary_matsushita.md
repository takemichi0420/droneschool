# autopilot_demo.py 仕様解説

## 概要
このスクリプトは、MAVLink通信を使用してドローン（ArduCopter）の自動飛行を制御するプログラムです。GUIDED モードでの離陸、複数のウェイポイントの巡回、RTL（Return to Launch）での自動帰還までを実行します。

## 主要な機能と関数

### ユーティリティ関数

1. **`to_quaternion(roll, pitch, yaw)`** (autopilot_demo.py:9-24)
   - オイラー角（ロール・ピッチ・ヨー）をクォータニオンに変換
   - 姿勢制御で使用

2. **`wait_mode(master, mode, timeout)`** (autopilot_demo.py:27-44)
   - 指定したフライトモードへの切り替えを待機
   - HEARTBEATメッセージを監視して確認

3. **`wait_altitude(master, target_alt, tolerance, timeout)`** (autopilot_demo.py:47-58)
   - 目標高度に到達するまで待機
   - GLOBAL_POSITION_INTメッセージから現在高度を取得

4. **`wait_position(master, lat, lon, alt, ...)`** (autopilot_demo.py:61-86)
   - 指定した緯度・経度・高度に到達するまで待機
   - デフォルトの位置許容誤差: 緯度経度 0.00005度（約5.5m）、高度 0.5m

5. **`wait_disarm(master, timeout)`** (autopilot_demo.py:89-98)
   - モーターのディスアーム（停止）を待機

6. **`send_attitude(master, roll, pitch, yaw, thrust, duration)`** (autopilot_demo.py:101-118)
   - 指定した姿勢とスラストを一定時間維持
   - 0.1秒間隔で繰り返し送信

7. **`set_rtl_altitude(master, altitude_m, timeout)`** (autopilot_demo.py:121-142)
   - RTL高度パラメータ（RTL_ALT）を設定
   - メートル単位を内部的にセンチメートルに変換

8. **`wait_command_ack(master, command, timeout)`** (autopilot_demo.py:145-156)
   - コマンドの実行結果（ACK）を待機
   - 結果コードと文字列表現を返す

## メインフロー（`main()`関数）

### 1. 接続 (autopilot_demo.py:162-166)
- `127.0.0.1:14551` に接続（SITL シミュレータを想定）
- ハートビート待機で接続確認

### 2. GUIDEDモードへ切替 (autopilot_demo.py:168-177)
- 外部からの位置指令を受け付けるモードに変更

### 3. アームと離陸 (autopilot_demo.py:179-218)
- モーターをアーム（起動）
- 目標高度: **10m**
- MAV_CMD_NAV_TAKEOFFコマンドを送信
- GLOBAL_POSITION_INTメッセージを5Hzで受信設定
- 目標高度到達まで待機

### 4. 巡航速度設定 (autopilot_demo.py:220-227)
- WPNAV_SPEEDパラメータを300 cm/s（**3 m/s**）に設定

### 5. ウェイポイント巡回 (autopilot_demo.py:231-264)
- **9個のウェイポイント**を順次巡回
- 座標例: (35.8792536, 140.339216, 10) など
- 各ウェイポイントに到達後、**5秒待機**
- `set_position_target_global_int_send()` で位置指令を送信

### 6. RTL帰還 (autopilot_demo.py:265-285)
- 全ウェイポイント巡回後、**10秒待機**
- RTL高度を**50m**に設定
- RTLモードに切替
- 離陸ポイントへ自動帰還し、着陸・ディスアームを待機

### 7. 終了 (autopilot_demo.py:287-288)
- 接続をクローズ

## 技術的なポイント

- **MAVLinkプロトコル**: pymavlink ライブラリを使用
- **座標系**: `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`（GPS座標、離陸地点からの相対高度）
- **型マスク**: `0b0000111111111000` で速度・加速度を無視し、位置のみ指令
- **エラーハンドリング**: 各ステップで失敗時は `sys.exit(1)` で終了
- **タイムアウト**: 各待機関数にタイムアウト設定あり

## まとめ

このスクリプトは、ドローンの基本的な自動飛行パターン（離陸→ウェイポイント巡回→帰還）を実装した教育・テスト用のデモコードです。MAVLinkプロトコルを使用した実用的な制御フローの例として、ドローン開発の学習に適しています。
