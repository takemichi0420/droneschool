# poi.py 仕様解説

## 概要
このスクリプトは、MAVLink通信を使用して、構造物を周回撮影（POI: Point of Interest）するドローンミッションを自動生成・実行するプログラムです。Gazebo シミュレータ上の物体位置を指定し、その周りを複数の高度で円軌道を描きながら撮影します。

## 主要な構成要素

### 1. データ構造

**`StructureSpec`** (poi.py:12-18)
- 構造物の仕様を定義するデータクラス
- `center_lat`, `center_lon`: 構造物の中心座標（緯度・経度）
- `width_m`, `depth_m`: 構造物の幅と奥行き（メートル）
- `base_alt_m`: 基準高度
- `height_m`: 構造物の高さ

**`GazeboOffset`** (poi.py:24-26)
- Gazebo座標系でのオフセット
- `x_m`: 東方向の距離（メートル）
- `y_m`: 北方向の距離（メートル）

### 2. 座標変換

**`gazebo_xy_to_latlon(home_lat, home_lon, offset)`** (poi.py:29-49)
- Gazeboの相対座標（X: 東、Y: 北）を緯度・経度に変換
- WGS84地球半径（6378137.0m）を使用した近似計算
- ホーム位置からのオフセットを考慮

### 3. ウェイポイント生成

**`estimate_orbit_radius(spec, safety_margin_m, min_radius_m)`** (poi.py:55-63)
- 構造物を包み込む最小円の半径を計算
- 構造物の対角線の半分に安全マージンを追加
- 最小半径を保証

**`plan_vertical_levels(spec, start_alt_m, end_alt_margin_m, alt_step_m, max_rings)`** (poi.py:66-88)
- 垂直方向の高度リングを計画
- 構造物の基準高度から頂上まで、指定間隔で高度を生成
- 最大リング数を制限（デフォルト30）

**`make_circle_points(center_lat, center_lon, radius_m, n_points)`** (poi.py:91-114)
- 指定した中心点の周りに円周上の点を生成
- 緯度・経度での座標リストを返す
- n_points個の等間隔な点を配置

**`rotate_points_to_closest(circle_pts, cur_lat, cur_lon)`** (poi.py:172-189)
- 現在地に最も近い点から始まるように円周上の点を回転
- 構造物との衝突回避に使用

**`plan_structure_orbits(spec, cur_lat, cur_lon, ...)`** (poi.py:192-214)
- 構造物周回軌道全体を計画
- 複数高度のリングを生成
- 各リングは(緯度, 経度, 高度)のタプルリスト

### 4. ミッション生成

**`make_mission_item_int(master, seq, ...)`** (poi.py:220-241)
- MAVLink ミッションアイテムを生成するヘルパー関数
- MISSION_ITEM_INT メッセージを構築

**`build_orbit_mission(master, spec, ...)`** (poi.py:244-339)
ミッション全体を構築:
1. **TAKEOFF**: 最下段の高度まで離陸 (poi.py:267-282)
2. **DO_SET_ROI**: 構造物の中心（高さの中間点）を注視 (poi.py:284-300)
   - `MAV_ROI_LOCATION` モードで構造物中心に向ける
3. **WAYPOINT**: 各リングの円周上の全ポイントを巡回 (poi.py:302-321)
   - `param2=2.0`: 通過半径2メートル
4. **RTL**: 最後に離陸地点へ帰還 (poi.py:323-334)

### 5. ミッションアップロード

**`upload_mission(master, mission_items)`** (poi.py:345-405)
- 既存ミッションをクリア
- `MISSION_COUNT` を送信
- `MISSION_REQUEST`/`MISSION_REQUEST_INT` に応答して各アイテムを送信
- `MISSION_ACK` を待って完了確認
- シーケンス0を現在のミッションに設定

### 6. 飛行制御関数

**`set_rtl_params(master, rtl_alt_m, land_speed_cms)`** (poi.py:411-460)
- RTL（Return to Launch）パラメータを設定:
  - `RTL_ALT`: RTL時の高度（cm単位）
  - `RTL_ALT_FINAL`: 最終高度（0）
  - `RTL_AUTOLAND`: 自動着陸有効
  - `LAND_ENABLE`: 着陸モード有効
  - `LAND_SPEED`: 着陸速度（cm/s）

**`set_mode_blocking(master, mode_name)`** (poi.py:463-487)
- フライトモードを変更し、完了まで待機
- HEARTBEATメッセージで確認

**`disable_arming_check(master)`** (poi.py:490-502)
- アーミングチェックを無効化（シミュレーション用）

**`wait_for_position(master, timeout)`** (poi.py:516-541)
- GLOBAL_POSITION_INTメッセージを受信するまで待機
- "Need Position Estimate" エラーを回避

**`guided_takeoff(master, alt)`** (poi.py:543-575)
- GUIDEDモードで離陸を実行:
  1. GUIDEDモードに切替
  2. モーターをアーム
  3. MAV_CMD_NAV_TAKEOFFコマンドを送信
  4. 高度を監視

**`arm_and_start_auto(master, takeoff_alt)`** (poi.py:577-607)
- AUTO ミッション全体を開始:
  1. RTLパラメータ設定
  2. 位置推定の安定化待ち
  3. GUIDED で離陸
  4. AUTO モードに切替
  5. MAV_CMD_MISSION_START を送信

### 7. ユーティリティ関数

**`get_home_position(master, timeout)`** (poi.py:117-139)
- HOME_POSITIONメッセージからホーム座標を取得
- MAV_CMD_GET_HOME_POSITIONコマンドを使用

**`get_current_latlon(master, timeout)`** (poi.py:142-155)
- GLOBAL_POSITION_INTから現在位置を取得

**`approx_dist_m(lat1, lon1, lat2, lon2)`** (poi.py:158-169)
- 2点間の距離を平面近似で計算

**`print_statustext(master, timeout)`** (poi.py:505-513)
- STATUSTEXTメッセージを表示

## メインフロー（`main()`関数）

### 実行手順 (poi.py:613-653)

1. **機体接続** (poi.py:615-618)
   - `udp:127.0.0.1:14550` に接続
   - ハートビート待機

2. **Gazebo物体位置設定** (poi.py:620-628)
   - オフセット: X=1.0m（東）、Y=1.0m（北）
   - ホーム位置から緯度経度を計算

3. **構造物スペック定義** (poi.py:630-637)
   - 幅: 1.0m、奥行き: 1.0m
   - 基準高度: 0.0m、高さ: 4.0m

4. **ミッション構築** (poi.py:640-646)
   - リングあたり10ポイント
   - 安全マージン: 1.0m
   - 最小半径: 1.0m

5. **ミッションアップロード** (poi.py:649)

6. **AUTO開始** (poi.py:652)
   - GUIDED離陸後、AUTOモードでミッション実行

## 技術的なポイント

- **POI撮影**: `DO_SET_ROI`コマンドで常に構造物中心を注視
- **円軌道飛行**: 複数高度で構造物を周回し、全方位から撮影
- **衝突回避**: 現在地から最も近い点をスタート地点に設定
- **柔軟な設計**: 構造物サイズに応じて自動的に軌道半径を調整
- **プロトコル**: MAVLink MISSION_ITEM_INT を使用した高精度座標
- **座標系**: `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`（GPS座標、相対高度）

## 実用例

デフォルト設定での動作:
- 1m × 1m × 4m の構造物
- ホームから東へ1m、北へ1m の位置
- 安全半径約1.7m（対角線+マージン）
- 10ポイント/リング × 複数高度
- 構造物中心を常に注視しながら周回撮影

## まとめ

このスクリプトは、構造物の3次元スキャンや点検を自動化するための高度なミッション生成システムです。Gazeboシミュレータでの検証を前提とし、実際の現場でのドローン点検業務に応用可能な設計となっています。
