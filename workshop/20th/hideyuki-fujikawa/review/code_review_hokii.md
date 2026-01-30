# コードレビュー: poi.py

**ファイル:** `workshop/20th/takao_hokii/poi.py`
**レビュアー:** Claude Code
**日付:** 2025-12-18
**コード行数:** 656行

---

## 総括

本レビューでは、構造物の自動点検を行うためのMAVLinkベースのPOI（Point of Interest）ミッション生成プログラムを検証しました。このコードは、構造物の周囲を複数の高度で円軌道飛行し、常にカメラを対象物に向けるミッションを生成します。

**全体評価:** MAVLinkプロトコルとミッション計画について優れた理解を示していますが、クラッシュや安全上の問題を引き起こす可能性のある**5つの重大な問題**と、信頼性に影響する11の中程度の問題が含まれています。

**発見された問題総数:** 20件（高: 5、中: 11、低: 4）

---

## 重大な問題（High Severity）

### 1. インデックス範囲外エラーのリスク
**箇所:** 268行目
**深刻度:** 🔴 高

```python
takeoff_alt = rings[0][0][2]  # 最下段の高度まで上げる
```

**問題点:**
- `rings`リストが空でないことを確認せずに`rings[0][0][2]`にアクセスしています
- `plan_structure_orbits()`が空のringsリストを返すと`IndexError`が発生します

**影響:** ミッション構築中にプログラムがクラッシュします

**推奨対策:**
```python
if not rings or not rings[0]:
    raise RuntimeError("ミッション用のリングが生成されませんでした")
takeoff_alt = rings[0][0][2]
```

---

### 2. アーミングチェック無効化の安全性の不整合
**箇所:** 490-502行目、587行目
**深刻度:** 🔴 高

```python
def disable_arming_check(master):
    """ARMING_CHECK をオフにする。"""
    # ... 実装が存在 ...

# しかし arm_and_start_auto() では:
# disable_arming_check(master)  # コメントアウトされている！
```

**問題点:**
- 安全チェックを無効化する関数が存在するがコメントアウトされています
- 安全チェックをバイパスすることが意図されているのか不明確です
- 誤ってコメント解除されると、重要なフライト前チェックが無効化されます

**影響:** コメント解除時の安全上の危険性、設計意図の混乱

**推奨対策:**
- **オプションA:** シミュレーションで不要な場合は関数を完全に削除
- **オプションB:** なぜ存在するのか、いつ使用すべきかを明確に文書化
- **オプションC:** この動作を明示的に制御する設定パラメータを追加

---

### 3. パラメータ設定の確認なし
**箇所:** 420-458行目
**深刻度:** 🔴 高

```python
def set_rtl_params(master, rtl_alt_m=10.0, land_speed_cms=50):
    master.mav.param_set_send(...)  # 複数回呼び出し
    # ...
    time.sleep(1.0)  # 待機するだけで確認していない！
```

**問題点:**
- RTLおよび着陸パラメータがfire-and-forget方式で送信されています
- `PARAM_VALUE`応答の受信/確認がありません
- パラメータ設定が失敗しても検出されず、RTL動作が不正確で危険になります

**影響:** 重要な安全パラメータが設定されない可能性があり、不正確なReturn-to-Launch動作を引き起こします

**推奨対策:**
```python
def set_rtl_params(master, rtl_alt_m=10.0, land_speed_cms=50):
    params = [
        ("RTL_ALT", rtl_alt_m * 100, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
        ("RTL_ALT_FINAL", 0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
        ("RTL_AUTOLAND", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT8),
        ("LAND_ENABLE", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT8),
        ("LAND_SPEED", land_speed_cms, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    ]

    for param_id, value, param_type in params:
        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            param_id.encode(),
            value,
            param_type
        )

        # 確認を待つ
        t0 = time.time()
        confirmed = False
        while time.time() - t0 < 5.0:
            msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
            if msg:
                recv_id = msg.param_id
                if isinstance(recv_id, bytes):
                    recv_id = recv_id.decode('utf-8', errors='ignore')
                recv_id = recv_id.strip('\x00')
                if recv_id == param_id:
                    print(f"[PARAM] {param_id} を {value} に設定しました")
                    confirmed = True
                    break

        if not confirmed:
            raise RuntimeError(f"パラメータ {param_id} の設定に失敗しました")
```

---

### 4. モード変更の無限ループリスク
**箇所:** 478-487行目
**深刻度:** 🔴 高

```python
def set_mode_blocking(master, mode_name: str):
    master.mav.set_mode_send(...)

    while True:  # 無限ループ！
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        if not msg:
            print("[MODE] waiting HEARTBEAT...")
            continue
        current_mode = mavutil.mode_string_v10(msg)
        if current_mode == mode_name:
            break
```

**問題点:**
- 最大反復カウンターがありません
- モード変更が失敗した場合（ハードウェアの問題、無効なモードなど）、ループが永久に実行されます
- プログラムが無期限にフリーズします

**影響:** プログラムフリーズにより手動終了が必要になります

**推奨対策:**
```python
def set_mode_blocking(master, mode_name: str, max_attempts=20):
    modes = master.mode_mapping()
    if mode_name not in modes:
        raise RuntimeError(f"モード {mode_name} が mode_mapping() に存在しません: {modes}")
    mode_id = modes[mode_name]

    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

    attempts = 0
    while attempts < max_attempts:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        if not msg:
            print("[MODE] HEARTBEAT を待機中...")
            attempts += 1
            continue
        current_mode = mavutil.mode_string_v10(msg)
        if current_mode == mode_name:
            print(f"[MODE] -> {current_mode}")
            return True
        attempts += 1

    raise RuntimeError(f"{max_attempts}回の試行後、モード {mode_name} への変更に失敗しました")
```

---

### 5. 位置情報取得失敗時の黙認
**箇所:** 516-541行目
**深刻度:** 🔴 高

```python
def wait_for_position(master, timeout=30.0):
    # ... GLOBAL_POSITION_INT を取得するループ ...

    if not got_pos:
        print("[POS] WARNING: no GLOBAL_POSITION_INT within timeout")
        # エラーを発生させずに返す！
```

**問題点:**
- 警告が表示されますが、有効な位置データなしで実行が継続されます
- 後で呼び出される`get_current_latlon()`（252行目）が失敗するか、古いデータを返します
- 無効な座標でミッション計画が進行します

**影響:** 不正確な座標でミッションが生成され、機体の飛び去りの可能性があります

**推奨対策:**
```python
if not got_pos:
    raise RuntimeError(
        "タイムアウト内に位置推定を取得できませんでした。"
        "ミッション計画を続行できません。"
    )
```

---

## 中程度の問題（Medium Severity）

### 6. 高度フレームの解釈が不明確
**箇所:** 272、289、308、327行目
**深刻度:** 🟡 中

```python
frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
```

**問題点:**
- すべてのミッションアイテムが相対高度フレームを使用しています
- `z`値はホーム/地面からの高度です
- `spec.base_alt_m`フィールドが存在しますが、相対高度との関係が不明確です
- 地面が0mでない場合、高度の解釈が曖昧になります

**推奨対策:**
- 相対高度ミッションの場合は`spec.base_alt_m`を0にすべきことを文書化する
- または、構造物が非水平地形にある場合は絶対高度フレームに変換する

---

### 7. MISSION_ACKエラーコードの検証なし
**箇所:** 394-400行目
**深刻度:** 🟡 中

```python
elif mtype == 'MISSION_ACK':
    print(f"[MISSION] ACK received: type={msg.type}, sent={sent}/{n}")
    if sent < n:
        print("[MISSION] premature ACK (probably from clear_all), ignore and continue")
        continue
    else:
        break
```

**問題点:**
- `msg.type`フィールドに結果コードが含まれていますが、表示されるだけでチェックされていません
- `msg.type == MAV_MISSION_RESULT.MISSION_ACCEPTED` (0)を確認すべきです
- 他のコードはエラーを示します: INVALID、UNSUPPORTEDなど

**推奨対策:**
```python
elif mtype == 'MISSION_ACK':
    if msg.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
        raise RuntimeError(f"ミッションアップロード失敗: コード={msg.type}")
    if sent >= n:
        print(f"[MISSION] {sent}/{n} アイテムをアップロードしました")
        break
```

---

### 8. 離陸完了の確認なし
**箇所:** 543-575行目
**深刻度:** 🟡 中

```python
def guided_takeoff(master, alt=1.0):
    # ... 離陸コマンドを送信 ...

    t0 = time.time()
    while time.time() - t0 < 20:
        # 高度を監視するが、目標に到達したかチェックしない
        msg = master.recv_match(...)
        if msg and msg.get_type() == 'GLOBAL_POSITION_INT':
            rel_alt = msg.relative_alt / 1000.0
            print(f"[POS] rel_alt={rel_alt:.1f} m")
        time.sleep(0.5)
    # 離陸が完了したかどうかに関わらず返る！
```

**問題点:**
- 関数は20秒待ってから返ります
- ドローンが実際に目標高度に到達したことを確認していません
- 後続のAUTOミッションがドローンが上昇中に開始される可能性があります

**推奨対策:**
```python
def guided_takeoff(master, alt=1.0, tolerance=0.5):
    set_mode_blocking(master, "GUIDED")

    print("[ARM] アーム要求...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print("[ARM] アーム完了")

    print(f"[TAKEOFF] {alt} m まで離陸中（GUIDED）")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0,
        alt
    )

    t0 = time.time()
    while time.time() - t0 < 60:  # より長いタイムアウト
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            rel_alt = msg.relative_alt / 1000.0
            print(f"[POS] rel_alt={rel_alt:.1f} m")
            if rel_alt >= alt - tolerance:
                print(f"[TAKEOFF] {alt}m に到達しました")
                return True
        time.sleep(0.5)

    raise RuntimeError(f"60秒以内に離陸高度 {alt}m に到達できませんでした")
```

---

### 9. ホーム位置座標の検証なし
**箇所:** 627-628行目
**深刻度:** 🟡 中

```python
home_lat, home_lon, _ = get_home_position(master)
obj_lat, obj_lon = gazebo_xy_to_latlon(home_lat, home_lon, offset)
```

**問題点:**
- 緯度/経度値の範囲チェックがありません
- 無効な座標（0, 0）やNaNが計算を通して静かに伝播します
- 無効なGPS座標でミッションが生成されます

**推奨対策:**
```python
home_lat, home_lon, _ = get_home_position(master)

if not (-90 <= home_lat <= 90):
    raise ValueError(f"無効なホーム緯度: {home_lat}")
if not (-180 <= home_lon <= 180):
    raise ValueError(f"無効なホーム経度: {home_lon}")

obj_lat, obj_lon = gazebo_xy_to_latlon(home_lat, home_lon, offset)
```

---

### 10. 衝突回避の前提条件が検証されていない
**箇所:** 192-214行目
**深刻度:** 🟡 中

**問題点:**
- `rotate_points_to_closest()`はドローンが`cur_lat, cur_lon`から開始することを前提としています
- ドローン位置がホームからずれている場合、軌道がドローンを意図したものより構造物に近づける可能性があります
- 安全マージンの計算はミッション構築時に一度だけ行われます

**推奨対策:**
- ドローンがミッション開始前にホーム位置にいる必要があることを文書化する
- 距離チェックを追加: AUTO開始前にドローンがホームの合理的な範囲内にあることを確認する

---

### 11. メッセージ受信の再試行ロジックなし
**箇所:** 131、148、379行目
**深刻度:** 🟡 中

**問題点:**
- `HOME_POSITION`、`GLOBAL_POSITION_INT`、`MISSION_REQUEST`に対する単一タイムアウト試行のみです
- ネットワークの不具合やオートパイロットの応答遅延が即座の失敗を引き起こします
- 指数バックオフや再試行メカニズムがありません

**推奨対策:**
```python
def get_home_position(master, timeout=10.0, retries=3):
    """HOME_POSITIONメッセージからホームの緯度経度を取得する（再試行付き）。"""
    for attempt in range(retries):
        try:
            t0 = time.time()
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
                0, 0, 0, 0, 0, 0, 0, 0
            )

            while time.time() - t0 < timeout:
                msg = master.recv_match(type='HOME_POSITION', blocking=True, timeout=1)
                if msg:
                    lat = msg.latitude / 1e7
                    lon = msg.longitude / 1e7
                    alt = msg.altitude / 1000.0
                    return lat, lon, alt
        except Exception as e:
            if attempt == retries - 1:
                raise
            print(f"[RETRY] 試行 {attempt + 1} 失敗、再試行中...")
            time.sleep(0.5)

    raise RuntimeError("再試行後もHOME_POSITIONの取得に失敗しました")
```

---

### 12. 空のミッションが黙って返される
**箇所:** 351-353行目
**深刻度:** 🟡 中

```python
if n == 0:
    print("[MISSION] empty mission, skip")
    return
```

**問題点:**
- 何もアップロードせずに黙って返ります
- ドローンはメモリ内の以前のミッションを続行します
- 呼び出し側はアップロードがスキップされたことを知りません

**推奨対策:**
```python
if n == 0:
    raise ValueError("空のミッションをアップロードできません")
```

---

### 13. ゼロ除算のリスク
**箇所:** 80行目
**深刻度:** 🟡 中

```python
n_rings = int(math.floor((top_alt - bottom_alt) / alt_step_m)) + 1
```

**問題点:**
- `alt_step_m`が0または負の場合、`ZeroDivisionError`が発生します
- 関数パラメータの入力検証がありません

**推奨対策:**
```python
def plan_vertical_levels(spec: StructureSpec,
                         start_alt_m: float = 1.0,
                         end_alt_margin_m: float = 0.0,
                         alt_step_m: float = 1.0,
                         max_rings: int = 30) -> List[float]:
    """
    start_alt_m から構造物高さまで alt_step_m 刻みで高度リングを作る。
    """
    if alt_step_m <= 0:
        raise ValueError(f"alt_step_mは正でなければなりません、入力値: {alt_step_m}")

    bottom_alt = spec.base_alt_m + start_alt_m
    top_alt = spec.base_alt_m + spec.height_m + end_alt_margin_m
    # ... 残りの処理 ...
```

---

### 14. ミッションプロトコルの競合状態
**箇所:** 358-364行目
**深刻度:** 🟡 中

```python
# 古いメッセージをフラッシュ
while True:
    msg = master.recv_match(
        type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'],
        blocking=False
    )
    if not msg:
        break
```

**問題点:**
- 非ブロッキングフラッシュは、まだ送信中のメッセージを見逃す可能性があります
- フラッシュ後に新しいメッセージが到着する小さなタイミングウィンドウがあります
- プロトコル状態の不一致を引き起こす可能性があります

**推奨対策:**
- フラッシュ後に小さな遅延を追加: `time.sleep(0.1)`
- またはシーケンス番号を使用してメッセージの順序を追跡する

---

### 15. マジックナンバー - 不正確なパラメータ使用
**箇所:** 203行目
**深刻度:** 🟡 中

```python
alt_list = plan_vertical_levels(spec, radius_m)
```

**問題点:**
- `radius_m`が`start_alt_m`パラメータ（specの後の最初の位置引数）として渡されています
- これはロジックエラーのように見えます - なぜ軌道半径が開始高度として使用されるのでしょうか？
- 関数シグネチャ: `plan_vertical_levels(spec, start_alt_m=1.0, ...)`

**推奨対策:**
ロジックを見直して、次のいずれかにする:
- `alt_list = plan_vertical_levels(spec, start_alt_m=1.0)`に変更
- または、なぜ半径が開始高度であるべきかを文書化する

---

### 16. ウェイポイントパラメータが不明確
**箇所:** 313行目
**深刻度:** 🟡 中

```python
p2=2.0,  # これは何を意味するのか？
```

**問題点:**
- `MAV_CMD_NAV_WAYPOINT`の場合、param2は受け入れ半径（メートル単位）です
- 2.0の値は、ドローンが2m以内になったときにウェイポイントを通過することを意味します
- コード内で文書化されていません

**推奨対策:**
```python
# モジュール定数として定義
WAYPOINT_ACCEPTANCE_RADIUS_M = 2.0  # ウェイポイントの受け入れ半径[m]

# build_orbit_mission()内で:
p2=WAYPOINT_ACCEPTANCE_RADIUS_M,  # ウェイポイントの受け入れ半径
```

---

## コード品質の問題（Low Severity）

### 17. マジックナンバー 1e7 の繰り返し
**箇所:** 134、150、277、295、316、317行目
**深刻度:** 🟢 低

**問題点:**
- MAVLink座標スケーリング係数（1e7）がコード全体で繰り返されています
- モジュールレベルの定数として定義すべきです

**推奨対策:**
```python
# ファイルの先頭で定義
MAVLINK_COORD_SCALE = 1e7  # MAVLinkは緯度経度に1e7スケール係数を持つint32を使用

# 使用例:
x=int(lat * MAVLINK_COORD_SCALE)
y=int(lon * MAVLINK_COORD_SCALE)
```

---

### 18. NaNパラメータの使用が不明確
**箇所:** 315行目
**深刻度:** 🟢 低

```python
p4=float("nan"),  # ヨー角
```

**問題点:**
- ヨーパラメータにNaNを使用するのは型破りです
- MAVLinkはこれを受け入れる可能性がありますが、動作が予測不可能な場合があります
- NaNが使用される理由を説明するコメントがありません

**推奨対策:**
```python
p4=float("nan"),  # NaN = 現在のヨーを使用、方向を変更しない
# またはより良い:
p4=0.0,  # ヨー角（0=北） - DO_SET_ROIによって上書きされる
```

---

### 19. 関数の複雑度が高い
**箇所:** 345-405行目（`upload_mission`）
**深刻度:** 🟢 低

**問題点:**
- 関数が複数の関心事を処理: クリア、カウント、送信、ACK処理
- ネストされたロジックで約60行
- 個々のコンポーネントをテストすることが困難です

**推奨対策:**
小さな関数に分割する:
```python
def _clear_existing_mission(master):
    """オートパイロット上の既存のミッションをクリアする。"""
    target_sys = master.target_system or 1
    target_comp = master.target_component
    master.mav.mission_clear_all_send(target_sys, target_comp)
    time.sleep(0.2)

def _send_mission_count(master, count):
    """ミッションアイテム数をオートパイロットに送信する。"""
    target_sys = master.target_system or 1
    target_comp = master.target_component
    master.mav.mission_count_send(target_sys, target_comp, count)
    print(f"[MISSION] count={count} 送信、リクエスト待機中...")

def _handle_mission_requests(master, mission_items, n):
    """MISSION_REQUESTメッセージを処理し、アイテムを送信する。"""
    sent = 0
    while True:
        msg = master.recv_match(
            type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'],
            blocking=True,
            timeout=30
        )
        if msg is None:
            raise RuntimeError("MISSION_REQUEST / MISSION_ACK の待機がタイムアウトしました")

        mtype = msg.get_type()

        if mtype in ('MISSION_REQUEST', 'MISSION_REQUEST_INT'):
            seq = msg.seq
            print(f"[MISSION] リクエスト seq={seq}")
            if not (0 <= seq < n):
                raise RuntimeError(f"無効なミッションリクエスト seq={seq}")
            master.mav.send(mission_items[seq])
            sent += 1

        elif mtype == 'MISSION_ACK':
            if msg.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise RuntimeError(f"ミッションアップロード失敗: コード={msg.type}")
            if sent >= n:
                print(f"[MISSION] {sent}/{n} アイテムをアップロードしました")
                return sent

def upload_mission(master, mission_items):
    """ミッションアイテムをオートパイロットにアップロードする。"""
    n = len(mission_items)
    if n == 0:
        raise ValueError("空のミッションをアップロードできません")

    target_sys = master.target_system or 1
    target_comp = master.target_component

    # 古いメッセージをフラッシュ
    while True:
        msg = master.recv_match(
            type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'],
            blocking=False
        )
        if not msg:
            break

    _clear_existing_mission(master)
    _send_mission_count(master, n)
    _handle_mission_requests(master, mission_items, n)

    master.mav.mission_set_current_send(target_sys, target_comp, 0)
    print("[MISSION] 現在のシーケンスを0に設定しました")
```

---

### 20. 座標スケーリングコードの重複
**箇所:** 277-278、295-296、316-317行目（エンコード）、134、150行目（デコード）
**深刻度:** 🟢 低

**問題点:**
- 座標エンコード/デコードパターンが複数回繰り返されています
- DRY原則に違反しています

**推奨対策:**
```python
def encode_latlon(lat: float, lon: float) -> tuple[int, int]:
    """緯度/経度（度）をMAVLink int32形式（1e7スケール）に変換する。"""
    return int(lat * MAVLINK_COORD_SCALE), int(lon * MAVLINK_COORD_SCALE)

def decode_latlon(x: int, y: int) -> tuple[float, float]:
    """MAVLink int32座標を緯度/経度（度）に変換する。"""
    return x / MAVLINK_COORD_SCALE, y / MAVLINK_COORD_SCALE

# 使用例:
x, y = encode_latlon(spec.center_lat, spec.center_lon)

# get_home_position での使用:
lat, lon = decode_latlon(msg.latitude, msg.longitude)
```

---

## 良い点

1. **構造が明確:** セクションごとに番号付けされた明確な関心の分離
2. **良好なドキュメント:** 日本語コメントが意図を明確に説明しています
3. **MAVLinkの知識:** ミッションプロトコルについての優れた理解を示しています
4. **実用的な設計:** POI軌道計画ロジックが堅実です
5. **エラーメッセージ:** デバッグに役立つ情報的なprintステートメント

---

## 推奨事項のまとめ

### 必須修正（重大 - High Severity）
1. ✅ `rings[0][0][2]`アクセスの境界チェックを追加
2. ✅ `disable_arming_check()`関数を削除または文書化
3. ✅ PARAM_VALUE確認の検証を実装
4. ✅ モード変更ループに最大反復カウンターを追加
5. ✅ 位置が取得できない場合は例外を発生させる

### 推奨修正（重要 - Medium Severity）
6. ✅ ホーム位置座標を検証
7. ✅ `guided_takeoff()`で高度到達を確認
8. ✅ MISSION_ACK結果コードを明示的にチェック
9. ✅ メッセージ受信に再試行ロジックを追加
10. ✅ 入力パラメータを検証（alt_step_m > 0など）
11. ✅ 空のミッションに対して黙って返すのではなくエラーを発生させる
12. ✅ `plan_vertical_levels()`呼び出しを正しいパラメータで修正

### あると良い（品質 - Low Severity）
13. ✅ `MAVLINK_COORD_SCALE`定数を定義
14. ✅ `encode_latlon()` / `decode_latlon()`ヘルパーを作成
15. ✅ `upload_mission()`を小さな関数に分割
16. ✅ NaNヨーパラメータの使用を文書化
17. ✅ 包括的なdocstringを追加

---

## 結論

このコードは優れたMAVLink知識を示し、有用なPOI点検パターンを実装しています。しかし、安全にデプロイする前に対処すべき**5つの重大な問題**があります:

- クラッシュのリスク（インデックスエラー、無限ループ）
- 黙って失敗（パラメータ、位置、ミッションACK）
- 安全上の懸念（アーミングチェック、離陸確認）

重大な問題に対処した後、このコードは優れた信頼性と保守性を備えた本番環境対応となります。

**総合評価:** B-（良好なコンセプトと構造、ただし信頼性の改善が必要）

---

**レビュー完了**
質問や説明が必要な場合は、コード作成者にご連絡ください。
