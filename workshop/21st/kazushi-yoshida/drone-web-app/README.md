# ドローン Web 制御アプリケーション (BlueOS Extension)

ドローンを Web ブラウザから操作する最小構成のアプリケーションです。
接続・アーム・離陸・着陸・モード変更・指定座標への移動を、状態をリアルタイムに確認しながら実行できます。

BlueOS Extension としてインストールして使うことを主眼にしていますが、
ローカル PC + ArduPilot SITL でも単体で動作します。

## 機能一覧

- 機体への接続(接続ボタン契機。サーバー起動時には自動接続しない)
- アーム / ディスアーム
- 離陸(目標高度指定)/ 着陸
- GoTo(緯度・経度・高度指定で移動)
- フライトモード変更(GUIDED / AUTO / RTL / LOITER / STABILIZE)
- リアルタイム状態表示(接続・アーム・モード・緯度・経度・高度・ヘディング)
- 地図表示(Leaflet + OpenStreetMap)で機体位置マーカーと飛行軌跡を表示

## 技術スタック

- バックエンド: Python 3.11 + FastAPI + WebSocket
- MAVLink 通信: pymavlink
- フロントエンド: 素の HTML + JavaScript + CSS(ビルドツール不要)
- 地図: Leaflet(同梱)+ OpenStreetMap タイル

## Docker イメージ

| 項目 | 値 |
| --- | --- |
| イメージ名 | `kyoshida0424/drone-web-app` |
| タグ | `latest` |
| 対応アーキテクチャ | `linux/amd64` / `linux/arm64`(Raspberry Pi 対応) |
| ベース | `python:3.11-slim` |
| 公開ポート | `9999/tcp` |

イメージは Docker Hub に公開済みです。BlueOS から直接 pull できます。

---

# BlueOS Extension として使う

## 前提条件

- BlueOS が動作していること(Raspberry Pi 等)
- **BlueOS が autopilot を認識していること**
  - Vehicle Setup → Autopilot 画面に機体が表示され、MAVLink Endpoints に機体データが流れている状態
  - 実機 FC が未接続の場合は、同画面でボードに **SITL** を選んで起動しておく
  - ここで機体が見えていないと、本アプリからは接続できません(下記トラブルシュート参照)
- ブラウザから BlueOS のホストへ TCP 9999 で到達できること

## インストール手順

BlueOS の **Extensions Manager → Installed → +(手動インストール)** から、以下を入力します。

| 項目 | 値 |
| --- | --- |
| Extension Identifier | `kyoshida0424.drone-web-app` |
| Extension Name | `Drone Web App` |
| Docker image | `kyoshida0424/drone-web-app` |
| Docker tag | `latest` |

**Custom settings** には次の JSON を貼り付けます。

```json
{
  "ExposedPorts": { "9999/tcp": {} },
  "Env": ["MAV_ENDPOINT=udpout:host.docker.internal:14550"],
  "HostConfig": {
    "PortBindings": { "9999/tcp": [{ "HostPort": "9999" }] },
    "ExtraHosts": ["host.docker.internal:host-gateway"]
  }
}
```

各設定の意味:

- `PortBindings` — WebSocket を使うため BlueOS のプロキシを経由せず、ホストのポート 9999 を直接開きます(`register_service` は `avoid_iframes: true` を返します)
- `ExtraHosts` — コンテナから BlueOS ホスト(MAVLink Router)へ到達するために必要です
- `Env: MAV_ENDPOINT` — 接続先。**BlueOS の MAVLink Endpoints 画面で、機体データが流れている UDP ポートに合わせて変更してください**

インストール後、BlueOS の左メニューに **Drone Web App** が現れます。
クリックすると `http://<BlueOS の IP>:9999/` が新しいタブで開きます。

## 接続先(MAV_ENDPOINT)の設定

接続先は環境変数 `MAV_ENDPOINT` で決まります(既定値は `udpout:host.docker.internal:14550`)。

| 値 | 用途 |
| --- | --- |
| `udpout:host.docker.internal:14550` | BlueOS の MAVLink Router 経由(既定) |
| `udpout:host.docker.internal:14551` | 上記で機体が見えない場合の別エンドポイント |
| `udpout:<PC の IP>:14551` | 別 PC で動かしている SITL に直接繋ぐ |
| `tcp:127.0.0.1:5762` | ローカル PC で SITL と同居させる場合 |

変更するには Custom settings の `Env` を書き換えて、拡張を **Restart** してください。
起動時に `MAVLink 接続先: ...` がログに出るので、**VIEW LOGS** で反映を確認できます。

> **注意:** 14550 は伝統的に地上局(QGC)向けのポートです。ここには GCS の HEARTBEAT しか流れていないことがあります。
> BlueOS の MAVLink Endpoints 画面で、機体テレメトリが出ているポートを確認して指定してください。

## 使い方

1. 左メニューの **Drone Web App** を開くと、サーバーへ自動で WebSocket 接続します(画面上部に「サーバー接続中」と表示)
2. **接続** ボタンを押すと、機体への MAVLink 接続を開始します(最大 30 秒待機)
3. 接続後、以下の操作が可能です
   - **アーム / ディスアーム** ボタンでモーターの armed 状態を切り替え
   - **離陸高度** を入力して **離陸**(内部で GUIDED に切替後、離陸)
   - **着陸** ボタンで着陸
   - **GoTo** に緯度・経度・高度を入力して **移動**(内部で GUIDED に切替)
   - **モード** ドロップダウンで選択し **設定** でモード変更
4. ステータスパネルと地図(マーカー・軌跡)がリアルタイムに更新されます

> 画面には接続表示が 2 つあります。「サーバー接続中」はブラウザ ↔ バックエンドの WebSocket、
> 「接続中 / 未接続」はバックエンド ↔ 機体の MAVLink です。後者が **接続** ボタンで張られます。

## トラブルシュート

### 「オートパイロットの HEARTBEAT を受信できませんでした」

エラーメッセージ末尾の文言で原因を切り分けられます。

| メッセージ | 原因 | 対処 |
| --- | --- | --- |
| ホスト名を解決できません | `ExtraHosts` 未設定 | Custom settings に `ExtraHosts` を追加して Restart |
| 到達できますが応答がありません | 指定ポートに何も流れていない | BlueOS の MAVLink Endpoints に該当ポートの UDP Server があるか確認 |
| MAVLink は届いていますが autopilot の HEARTBEAT がありません | **autopilot が不在**、または GCS 用ポートに接続している | 下記参照 |

3 つ目が最も多いパターンです。UDP は疎通していて MAVLink も届いているが、
届いているのは BlueOS 内部コンポーネント(mavlink-router 等)の HEARTBEAT だけ、という状態です。
これらは `MAV_AUTOPILOT_INVALID` を名乗るため、機体とは見なされません。

対処:

1. **BlueOS 側で機体が見えているか確認する** — Vehicle Setup → Autopilot に機体が出ていなければ、アプリ側では解決できません。実機 FC を接続するか、BlueOS の SITL を起動してください
2. **MAV_ENDPOINT のポートを変える** — MAVLink Endpoints 画面で機体データが流れているポートを指定
3. ポートを変えても症状が変わらない場合は、1 の可能性が濃厚です

### 地図が表示されない

地図タイルは OpenStreetMap から都度取得するため、インターネット接続が必要です。
Leaflet 本体は同梱しているため、オフラインでも UI 自体は表示されます。

---

# ローカル PC で動かす(SITL)

## 前提条件

- Python 3.11 系(FastAPI / Starlette の要件により 3.8 以上必須)
- ArduPilot SITL が `tcp:127.0.0.1:5762` で待ち受けていること
- ポート 9999 が空いていること

## 起動手順

```bash
cd backend
pip install -r requirements.txt

# 接続先を SITL に向ける(既定は BlueOS 向けのため上書きが必要)
export MAV_ENDPOINT=tcp:127.0.0.1:5762   # PowerShell: $env:MAV_ENDPOINT="tcp:127.0.0.1:5762"

uvicorn main:app --port 9999 --reload
```

ブラウザで <http://127.0.0.1:9999/> を開きます。

## Docker で動かす

```bash
# ビルド
docker build -t kyoshida0424/drone-web-app:latest .

# 実行
docker run --rm -p 9999:9999 \
  --add-host=host.docker.internal:host-gateway \
  -e MAV_ENDPOINT=udpout:host.docker.internal:14550 \
  kyoshida0424/drone-web-app:latest

# 公開
docker push kyoshida0424/drone-web-app:latest
```

BlueOS が動作する Raspberry Pi 向けには arm64 のイメージが必要です。
公開中のイメージは以下の buildx コマンドで amd64 / arm64 の両対応としてビルドしています。

```bash
docker buildx build --platform linux/arm64,linux/amd64 \
  -t kyoshida0424/drone-web-app:latest --push .
```

---

# 補足

## エンドポイント一覧

| パス | 種別 | 用途 |
| --- | --- | --- |
| `/` | GET | フロントエンド(index.html) |
| `/ws` | WebSocket | 状態配信とコマンド送信 |
| `/register_service` | GET | BlueOS 左メニューへの登録メタ情報 |
| `/docs` | GET | FastAPI の自動生成 API ドキュメント |

## WebSocket コマンド

| コマンド | 挙動 | 入力 |
| --- | --- | --- |
| `connect` | 未接続なら機体への接続を開始 | — |
| `arm` / `disarm` | `MAV_CMD_COMPONENT_ARM_DISARM` | `force`(任意) |
| `takeoff` | `MAV_CMD_NAV_TAKEOFF`。事前に GUIDED 切替 | `altitude` |
| `land` | `MAV_CMD_NAV_LAND` | — |
| `goto` | `set_position_target_global_int_send`。事前に GUIDED 切替 | `latitude` / `longitude` / `altitude` |
| `mode` | `set_mode()` でモード変更 | `mode` |

## 構成

```text
drone-web-app/
  README.md
  REQUIREMENTS.md
  Dockerfile
  backend/
    main.py
    requirements.txt
  frontend/
    index.html
    script.js
    style.css
    leaflet/          # Leaflet 同梱(オフライン動作用)
```

## 設計上のメモ

- 起動ポートは 9999 固定です
- MAVLink 受信のブロッキング処理は executor に逃がし、WebSocket / イベントループを止めません
- Router 経由では GCS 等の HEARTBEAT も混ざるため、autopilot の発生源のみを採用して状態を更新します
- `udpout` 接続では GCS heartbeat を約 1 秒ごとに送り、Router が返送先を維持できるようにしています
- `--no-access-log` で起動し、BlueOS のヘルスチェックによるログ肥大を防いでいます

## 安全上の注意

実機で試す場合は必ずプロペラを外し、認可された環境で行ってください。
本アプリには認証機構がないため、外部ネットワークへ公開しないでください。
