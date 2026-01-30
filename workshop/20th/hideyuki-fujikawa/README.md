# 第20回ドローンスクール - hideyuki-fujikawa

## 概要
ArduPilotとMAVLinkを使用したドローン制御のワークショップ課題です。
Node.js (TypeScript) と Python の両方で MAVLink 通信を実装しています。

## ディレクトリ構成

### nm_scripts/
Node.js + node-mavlink を使用した TypeScript スクリプト

- **nm01_message_dump.ts**: MAVLink メッセージのダンプとメッセージ間隔設定
  - 192.168.3.38:5762 に TCP 接続
  - GLOBAL_POSITION_INT メッセージを 10Hz で取得
  - 受信メッセージをコンソールに出力

- **nm02_arm_disarm_compact.ts**: ARM/DISARM コマンドの実装（コンパクト版）
  - メッセージダンプ機能に ARM コマンドを追加

- **nm02_arm_disarm_full.ts**: ARM/DISARM コマンドの完全版
  - フル実装版のスクリプト

### pm_scripts/
Python + pymavlink を使用したスクリプト

- **pm_scripts.py**: GUIDED モードでの離陸制御
  - 127.0.0.1:14551 に接続
  - GUIDED モードに設定
  - ARM してから 10m まで離陸
  - 高度到達まで監視

- **pm_scripts_control.py**: モード設定と離陸の簡易版
  - モード 4 (GUIDED) に設定
  - ARM して 10m 離陸コマンド送信

### multi_vehicles/
**複数機体順次制御システム（Python + pymavlink）**

複数の機体（ローバー、ボート、コプター）を順次制御するシステム。
前の機体がミッション完了後に次の機体が自動的に開始します。

- **sequential_control.py**: メインスクリプト
  - ローバー → ボート → コプター の順で制御
  - ミッションファイルからの自動読み込み・アップロード
  - 位置情報ベースのミッション完了判定
  - 各機体への接続、アーム、ミッション実行
  - コプターは離陸処理も実行

- **rover_mission.waypoints**: ローバー用ミッションファイル（QGC WPL 110形式）
- **boat_mission.waypoints**: ボート用ミッションファイル（QGC WPL 110形式）
- **copter_mission.waypoints**: コプター用ミッションファイル（QGC WPL 110形式）

- **README.md**: 詳細な使用方法とトラブルシューティング
- **multi_vehicle_prompt.log**: 開発過程の会話ログ

**主な機能:**
- ミッションファイル（.waypoints）からの自動読み込み
- MAVLinkプロトコルによるミッションアップロード
- 位置情報（GLOBAL_POSITION_INT）による確実なミッション完了判定
- 距離ベース判定（5m以内に5回連続で到達）
- 詳細なログ出力とエラーハンドリング

**実行方法:**
```bash
# SITLを起動（bat/multi_vehicle_dialog.bat）
cd bat
multi_vehicle_dialog.bat

# スクリプト実行
cd multi_vehicles
python3 sequential_control.py
```

### bat/
Windows バッチファイル（SITL起動用）

- **multi_vehicle.bat**: 複数機体SITL起動スクリプト
  - Rover (instance 0, sysid 1, port 5762)
  - Boat (instance 1, sysid 2, port 5772)
  - Copter (instance 2, sysid 3, port 5782)

- **multi_vehicle_dialog.bat**: 対話的SITL起動スクリプト
  - Mission Planner SITLパスを選択可能（ローカルPC or OneDrive）
  - 上記と同じ3機体を起動

### その他のファイル

- **pm_workshop.py**: Heartbeat 送信サンプル
  - 127.0.0.1:14551 に接続
  - 定期的に Heartbeat を送信

- **package.json**: Node.js プロジェクト設定
  - node-mavlink ^2.3.0 を使用

- **.gitignore**: Git 除外設定
  - node_modules, dump.log などを除外

## 実行方法

### Node.js スクリプト
```bash
npm install
npx ts-node nm_scripts/nm01_message_dump.ts
```

### Python スクリプト
```bash
python3 pm_workshop.py
python3 pm_scripts/pm_scripts.py
```

## 学習内容

### 基礎
- MAVLink プロトコルによるドローン通信
- ARM/DISARM コマンド
- モード切り替え (GUIDED, AUTO)
- 離陸制御
- メッセージ間隔設定
- 位置情報取得

### 応用（複数機体制御）
- 複数機体の同時SITL起動と管理
- QGC WPL 110形式のミッションファイル処理
- ミッションのアップロード/ダウンロード
- 位置情報（GLOBAL_POSITION_INT）による距離計算
- Haversine公式を使用した地理座標間の距離計算
- ミッション完了判定の実装
- 順次制御ロジックの構築
- エラーハンドリングとロバストな制御
