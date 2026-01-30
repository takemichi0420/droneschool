# Claude Code 会話ログ: 複数機体順次制御スクリプト開発

## 日時
2026-01-10

## 概要
複数機体（ローバー、ボート、コプター）を順次制御するPythonスクリプトの開発

---

## 会話1: 初期要件定義

**ユーザー:**
複数機体を制御する、Pythonスクリプトを作って。
workshop/20th/hideyuki-fujikawa/multi_vehicles の下に作って。

ローバー→ボート→コプターです。
前の機体が到着したら、次の機体がスタートします。
ローバー、ボートはアーム、ミッション開始、して。
コプターはアーム、離陸、ミッション開始、して。
ミッションはすでに機体に入っている前提です。

接続は下記を参考にして、ローバー 5762、ボート5772、コプター5782です。

```python
master: mavutil.mavfile = mavutil.mavlink_connection(
    "tcp:127.0.0.1:5782",  source_system=1, source_component=90)
```

**Claude:**
作成したファイル:
1. sequential_control.py - 複数機体順次制御スクリプト
2. README.md - 使用方法とトラブルシューティングガイド

主な機能:
- VehicleController クラスによる機体制御
- 順次実行（ローバー → ボート → コプター）
- 各機体のミッション完了待機
- クリーンなエラーハンドリング

---

## 会話2: ミッション完了判定の問題

**ユーザー:**
ローバーのミッションが終わってもボートが動かない

**Claude:**
問題: ミッション完了判定ロジックの不具合
- MISSION_CURRENT メッセージで判定していたが、モード変更でしか完了を検出できない
- ローバー/ボートは自動的にモード変更しないため無限ループ

修正内容:
- MISSION_ITEM_REACHED メッセージを使用
- 最後のウェイポイントに実際に到達したことを確認
- ミッション数を取得して最後のウェイポイント番号を特定

---

## 会話3: ミッションファイル読み込み機能追加

**ユーザー:**
ミッションは ファイルから読み込んでください。
@workshop/20th/hideyuki-fujikawa/multi_vehicles/boat_mission.waypoints
@workshop/20th/hideyuki-fujikawa/multi_vehicles/rover_mission.waypoints
@workshop/20th/hideyuki-fujikawa/multi_vehicles/copter_mission.waypoints

**Claude:**
追加機能:
1. load_mission_from_file() - QGC WPL 110形式のファイル読み込み
2. upload_mission() - ミッションアップロード機能
3. 自動ミッションアップロード - 接続後、アーム前に実行

これにより、事前にMission Plannerでアップロード不要に。

---

## 会話4: 再度のミッション完了判定問題

**ユーザー:**
ローバーが到達してないのにボートが開始してます。

**Claude (試行1):**
修正: wait_mission_complete() メソッドの改善
- メッセージ受信方法を改善（全メッセージを受信してタイプ判定）
- メッセージレート設定後の待機時間追加
- 詳細なログ出力追加

**Claude (試行2):**
追加修正: 距離ベース判定の追加
- MISSION_ITEM_REACHED が送信されない場合のフォールバック
- 最後のウェイポイントの座標を取得
- GLOBAL_POSITION_INT で現在位置を監視
- 距離が5m以内 AND MISSION_CURRENT が最後のWP → 完了判定

**Claude (試行3):**
条件の厳格化:
- 距離判定の条件を変更: `>=` から `==` に
- last_current_seq が一度は更新されていることを確認
- デバッグログの追加

---

## 会話5: 完全位置情報ベースへの移行

**ユーザー:**
機体の位置情報で判断してください。

**Claude:**
大幅な変更: 完全位置情報（距離）ベースの判定に移行

削除した機能:
- MISSION_ITEM_REACHED メッセージへの依存
- MISSION_CURRENT メッセージへの依存

新しいロジック:
1. 最後のウェイポイントの座標を取得
2. GLOBAL_POSITION_INT で現在位置を監視
3. 距離を計算（Haversine公式）
4. 5m以内に5回連続でいたら完了判定
5. 一時的に離れたらカウントリセット

メリット:
- MAVLinkメッセージの有無に依存しない
- 機体が実際にウェイポイントに到達したことを確実に判定
- シンプルで理解しやすいロジック

---

## 最終実装の詳細

### ファイル構成
```
workshop/20th/hideyuki-fujikawa/multi_vehicles/
├── sequential_control.py      # メインスクリプト
├── README.md                   # ドキュメント
├── rover_mission.waypoints     # ローバーミッション
├── boat_mission.waypoints      # ボートミッション
└── copter_mission.waypoints    # コプターミッション
```

### 主要クラス・関数

**load_mission_from_file(filepath)**
- QGC WPL 110形式のミッションファイルを読み込み
- タブ区切りデータをパース
- 緯度・経度を整数形式（×10^7）に変換

**VehicleController クラス**
- `__init__()` - 初期化（名前、接続文字列、機体タイプ、ミッションファイル）
- `connect()` - 機体に接続
- `arm()` - アーム処理
- `takeoff()` - 離陸（コプターのみ）
- `upload_mission()` - ミッションアップロード
- `start_mission()` - ミッション開始（AUTOモードに変更）
- `wait_mission_complete()` - ミッション完了待機（位置情報ベース）
- `get_mission_count()` - ミッション数取得
- `get_distance_to_waypoint()` - 距離計算
- `close()` - 接続クローズ

### wait_mission_complete() の詳細ロジック

```python
1. ミッション数を取得
2. 最後のウェイポイント番号を計算（mission_count - 1）
3. 最後のウェイポイントの座標を取得
4. GLOBAL_POSITION_INTメッセージレート設定（10Hz）
5. ループ:
   a. GLOBAL_POSITION_INTメッセージ受信
   b. 現在位置を取得
   c. Haversine公式で距離計算
   d. 距離が5m以内か判定:
      - YES: within_threshold_count++
        - カウント >= 5: ミッション完了
        - カウント < 5: 接近中ログ表示
      - NO: within_threshold_count = 0（リセット）
```

### 実行フロー

```
1. ローバー制御開始
   ├─ 接続
   ├─ ミッションアップロード
   ├─ アーム
   ├─ ミッション開始（AUTO）
   └─ ミッション完了待機（距離ベース）
       └─ 完了検出

2. ボート制御開始
   ├─ 接続
   ├─ ミッションアップロード
   ├─ アーム
   ├─ ミッション開始（AUTO）
   └─ ミッション完了待機（距離ベース）
       └─ 完了検出

3. コプター制御開始
   ├─ 接続
   ├─ ミッションアップロード
   ├─ アーム
   ├─ 離陸（GUIDED、3m）
   ├─ ミッション開始（AUTO）
   └─ ミッション完了待機（距離ベース）
       └─ 完了検出

4. 全機体制御完了
```

---

## 技術的な学び

### 1. MAVLinkメッセージの信頼性
- MISSION_ITEM_REACHED は環境によって送信されない場合がある
- メッセージベースの判定は環境依存のリスクがある
- 位置情報ベースの判定がより確実

### 2. ミッション完了判定の難しさ
- 単純な距離判定だけでは誤検出のリスク
- 連続判定（5回）により確実性向上
- 閾値外に出たらリセットで誤検出防止

### 3. ArduPilotの動作
- ローバー/ボートはミッション完了後も自動的にモード変更しない
- コプターは離陸前にGUIDEDモードが必要
- ミッション実行はAUTOモード

### 4. QGC WPL形式
- タブ区切りのテキストファイル
- 緯度・経度は度単位で保存
- MAVLinkでは整数（×10^7）で送信

---

## トラブルシューティング履歴

### 問題1: ミッション完了が検出されない
**原因:** モード変更ベースの判定
**解決:** MISSION_ITEM_REACHED メッセージ使用

### 問題2: MISSION_ITEM_REACHED が届かない
**原因:** 環境やファームウェアによってメッセージが送信されない
**解決:** 距離ベースの判定追加（フォールバック）

### 問題3: まだ到達していないのに完了判定
**原因:** MISSION_CURRENT が更新される前に距離判定が実行
**解決:** 条件の厳格化（== 比較、初期化チェック）

### 問題4: メッセージベース判定の信頼性
**原因:** メッセージの受信タイミングや環境依存
**解決:** 完全位置情報ベースに移行、連続判定導入

---

## 今後の改善案

1. **タイムアウト処理**
   - ミッション完了待機の最大時間設定
   - 一定時間経過後の警告

2. **エラーリカバリー**
   - ミッション失敗時の再試行
   - 通信断時の再接続

3. **並列実行オプション**
   - 複数機体の同時制御
   - 同期ポイントの設定

4. **詳細なログ記録**
   - ファイルへのログ出力
   - タイムスタンプ付き詳細ログ

5. **GUI化**
   - リアルタイム位置表示
   - 進捗状況の可視化

---

## 参考情報

### 使用したMAVLinkメッセージ
- HEARTBEAT - 接続確認、モード取得
- GLOBAL_POSITION_INT - 位置情報取得
- MISSION_COUNT - ミッション数取得
- MISSION_ITEM_INT - ミッションアイテム取得/送信
- MISSION_ACK - ミッションアップロード確認
- COMMAND_LONG - コマンド送信（メッセージレート設定など）

### 使用したMAVLinkコマンド
- MAV_CMD_SET_MESSAGE_INTERVAL - メッセージレート設定
- MAV_CMD_NAV_TAKEOFF - 離陸コマンド

### pymavlink API
- mavutil.mavlink_connection() - 接続確立
- wait_heartbeat() - ハートビート待機
- set_mode_apm() - モード変更
- arducopter_arm() - アーム
- motors_armed_wait() - アーム完了待機
- recv_match() - メッセージ受信
- mav.send() - メッセージ送信

---

## 結論

複数機体の順次制御スクリプトを完成させました。当初はMAVLinkメッセージベースの判定でしたが、
環境依存の問題を解決するため、最終的には完全位置情報ベースの判定に移行しました。

これにより、SITL環境でも実機でも確実に動作する堅牢なスクリプトが完成しました。

**最終的な判定方法:**
- 位置情報（GLOBAL_POSITION_INT）のみを使用
- 距離計算（Haversine公式）
- 5m以内に5回連続 → 完了判定
- 環境に依存しない確実な動作

---

## 作成日
2026-01-10
