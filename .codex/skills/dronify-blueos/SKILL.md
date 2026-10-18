---
name: dronify-blueos
description: ローカルで動く pymavlink + FastAPI のドローン Web アプリを BlueOS Extension（Docker化 + Extension化）に仕上げる。短い一問一答で方針を確定し、register_service・bridge permissions・host.docker.internal 接続・堅牢なモードマップ・受信フィルタ・Leaflet 同梱・Dockerfile を適用してローカルビルドまで検証する。push と BlueOS へのインストールは手動。drone web app を BlueOS 拡張として動かしたいときに使う。
---

# dronify-blueos

ローカル SITL で動く FastAPI 製ドローン Web アプリ（例: `drone-web-app`）を、**BlueOS Extension として動く状態**にする。AI が生成しただけのアプリでは満たされない BlueOS 固有の要件を、一問一答で確定 → 自動適用 → ローカル検証する。

**このスキルがやること:** 演習5（Docker化）と演習6（Extension化）の自動適用 ＋ ローカルビルド検証。
**やらないこと:** `docker push`（マルチアーキ）と BlueOS Web UI でのインストール（手順は最後に出力する）。

---

## フェーズ0: 一問一答（AskUserQuestion）

まず対象アプリのパスを特定する（`backend/main.py` が見つからなければユーザーに尋ねる）。
次に **AskUserQuestion** で以下を確認する（1〜2回の呼び出しにまとめてよい）。各回答が実装内容を一意に決める。

1. **アプリ名**（例: `Drone Web App`）
   → register_service の `name` / Docker image 名 / sanitized name を決定。
2. **リアルタイム方式**
   - `WebSocket`（推奨/既定）→ `avoid_iframes: true`（新ウィンドウで開く）。ポートは**固定**。
   - `HTTP ポーリング` → `works_in_relative_paths: true`（右ペイン埋め込み）。フロントは**相対パス**必須。
3. **ポート番号**（既定 `9999`。8080 は BlueOS の `mavlink-server` と衝突するので避ける）
4. **機体種別**（`Copter` / `Plane` / `Rover`）→ モード名検証に使用（例: GUIDED の存在）。
5. **地図（Leaflet）を使うか**（使う → ローカル同梱処理を行う）
6. **Docker Hub ユーザー名**（最後の push コマンド出力に使用。未定なら `<user>` のまま）

確定したら以降を自動で進める。

---

## フェーズ1: バックエンド（`backend/main.py`）に要件を適用

以下の4要件を満たすこと。すでに満たしていれば変更しない（冪等に）。

### ① 接続先を環境変数化（既定は BlueOS 用）
```python
import os
connection_string = os.environ.get("MAV_ENDPOINT", "udpout:host.docker.internal:14550")
```
- bridge では `localhost` で Router に届かないため **`host.docker.internal`**。`udpout`（client）で接続。
- ローカル確認は `MAV_ENDPOINT=tcp:127.0.0.1:5762` で上書きする運用。

### ② 非ブロックなバックグラウンド接続 ＋ autopilot 特定 ＋ 明示モードマップ
```python
import time, threading
master, MODE_MAP = None, {}

def _connect():
    global master, MODE_MAP
    while master is None:
        try:
            m = mavutil.mavlink_connection(connection_string)
            m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                 mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            # Router 経由は GCS の HEARTBEAT も混ざる。autopilot(非GCS) を特定する。
            hb = None
            deadline = time.time() + 30
            while time.time() < deadline:
                msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
                if msg and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                    hb = msg; break
            if hb is None:
                continue
            m.target_system, m.target_component = hb.get_srcSystem(), hb.get_srcComponent()
            # 機体タイプから明示生成（mode_mapping() は直近 HEARTBEAT を見て Plane/Copter を誤る）
            MODE_MAP = mavutil.mode_mapping_byname(hb.type) or {}
            m.mav.request_data_stream_send(m.target_system, m.target_component,
                                           mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
            master = m
        except Exception as e:
            print(f"connect retry: {e}"); time.sleep(3)

threading.Thread(target=_connect, daemon=True).start()
```
- **import 時に `wait_heartbeat()` でブロックしない**こと。ブロックすると uvicorn が起動せず `/register_service` も応答できず、左メニューに出ない。
- **`mode_mapping_byname(hb.type)` を使う**。Plane の `GUIDED=15` を Copter に送ると Copter ではモード15が **AUTOTUNE** になり「Mode change to Autotune failed」になる（Copter は `GUIDED=4`）。

### ③ 受信は自機のみ処理（mode/armed の点滅防止）＋ ARM 判定
受信ループ内で、`master.target_system/target_component` に一致するメッセージだけ処理する。
```python
if (msg and msg.get_srcSystem() == master.target_system
        and msg.get_srcComponent() == master.target_component):
    if msg.get_type() == "HEARTBEAT":
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)  # ← ARM 判定
        ...
```
- GCS（MAVProxy 等）の HEARTBEAT を拾うと `GUIDED⇔STABILIZE` 等で点滅する。
- ARM は `system_status==ACTIVE` ではなく **`base_mode & SAFETY_ARMED`** で判定する。

### ④ `/register_service`（リアルタイム方式で分岐）
WebSocket の場合（既定）:
```python
@app.get("/register_service")
async def register_service():
    return {
        "name": "<APP_NAME>",
        "description": "...",
        "icon": "mdi-drone",
        "company": "", "version": "1.0.0", "webpage": "", "api": "/docs",
        "avoid_iframes": True,   # WS はプロキシ非対応 → 直接 http://<IP>:<port>/ を開く
    }
```
HTTP ポーリングの場合は `avoid_iframes` を外し `"works_in_relative_paths": True` にする。
- 必須キー: `name/description/icon/company/version/webpage/api`。
- **`GET /` が 200 を返すこと**（index.html 配信で満たす。無ければ最小トップページを足す）。Helper は 200 でないとサービスを無効と見なし `/register_service` を呼ばない。

> HTTP ポーリングを選んだ場合のみ、フロントの絶対パス（`/static`、`fetch("/x")`）を**相対パス**（`static`、`fetch("x")`）に直す。WebSocket + avoid_iframes なら直さない（ルート配信なので絶対パスのまま動く）。

---

## フェーズ2: フロントエンド（地図を使う場合のみ）

Leaflet を使うなら **CDN 依存を排除**する。BlueOS のホットスポット接続中はインターネットに出られず `L is not defined` で地図が出ないため。

`templates/fetch-leaflet.sh` を `frontend/` で実行して `frontend/leaflet/`（js/css/images）に同梱し、`index.html` の unpkg 参照を `/static/leaflet/...`（ポーリング時は `static/leaflet/...`）へ差し替える。
地図**タイル**（OSM）のオフライン化はスコープ外（灰色背景＋マーカーで可）。

---

## フェーズ3: Docker 化

- `templates/Dockerfile` を雛形に、`<PORT>` を確定ポートに置換してアプリ直下に配置。
  - `permissions` LABEL は **bridge + PortBindings**。avoid_iframes（直接ポートを開く）なら `HostPort` を**固定**（`"<PORT>"`）、ポーリング/埋め込みなら自動（`""`）でよい。
  - `ExtraHosts` に `host.docker.internal:host-gateway`。
  - `CMD` の uvicorn に **`--no-access-log`**（Helper の継続ヘルスチェックでログ肥大 → VIEW LOGS タイムアウトを防ぐ）。
- `.dockerignore`（`__pycache__/`, `*.pyc`, `.git`, `*.md`）を置く。
- リポジトリに `.gitignore` が無ければ Python ＋ SITL/MAVProxy 生成物（`mav.tlog*`, `mav.parm`, `eeprom.bin`, `*.BIN`, `logs/`, `terrain/`）を追加。

---

## フェーズ4: ローカル検証（自動）

```bash
docker build -t <image> .
docker run -d --rm --name dronify-test -p <PORT>:<PORT> -e MAV_ENDPOINT=tcp:127.0.0.1:1 <image>
sleep 3
curl -s -o /dev/null -w "%{http_code}\n" http://localhost:<PORT>/                 # 200 を期待
curl -s http://localhost:<PORT>/register_service                                  # 妥当な JSON を期待
docker inspect <image> --format '{{ index .Config.Labels "permissions" }}'        # bridge/Port/ExtraHosts を確認
docker stop dronify-test
```
- `GET /` が 200、`/register_service` が必須キーを含むこと、`permissions` LABEL が妥当なことを確認する。
- SITL があれば `--network host -e MAV_ENDPOINT=tcp:127.0.0.1:5762` で実テレメトリも確認する。

---

## フェーズ5: 出力（push と install は手動）

最後に、ユーザーが手で行う手順を提示する（**実行はしない**）。

**マルチアーキ build & push**
```bash
docker buildx build --platform linux/amd64,linux/arm64 --provenance=false \
  -t <user>/<image>:latest --push .
```
（`blob upload unknown` が出たら再実行 or `--provenance=false` のまま）

**BlueOS インストール（Create from scratch）**
- Identifier `<user>.<sanitized-name>` / Name `<APP_NAME>` / Docker image `<user>/<image>` / tag `latest`
- **JSON エディタに permissions（LABEL と同じ）を必ず入力**（空 `{}` だとポート未マップでメニューに出ない）

**Definition of Done**
- [ ] 左メニューに表示／クリックで開く（avoid_iframes なら `:<PORT>` 直接）
- [ ] ステータスが点滅せず安定
- [ ] ARM / TAKEOFF（GUIDED に入る）/ LAND など操作が機能
- [ ] （地図あり）オフラインでもマーカー・航跡が描画

---

## チェックリスト（適用漏れ防止）

- [ ] `MAV_ENDPOINT` 環境変数（既定 `udpout:host.docker.internal:14550`）
- [ ] 非ブロック接続（バックグラウンドスレッド）
- [ ] autopilot 特定 ＋ `mode_mapping_byname`
- [ ] 受信を自機（target sys/comp）に限定
- [ ] ARM は `base_mode & SAFETY_ARMED`
- [ ] `/register_service`（WS→`avoid_iframes`、ポーリング→`works_in_relative_paths`）
- [ ] `GET /` が 200
- [ ] （地図）Leaflet ローカル同梱
- [ ] Dockerfile：permissions LABEL（bridge/Port/ExtraHosts）＋ `--no-access-log`
- [ ] ローカルビルド検証 OK
