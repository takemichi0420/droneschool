# Drone Web Control App

FastAPI + WebSocket + pymavlink で構成したドローン Web アプリです。BlueOS Extension として動作するように Docker 化と Extension 対応を入れており、ブラウザから機体の状態確認、アーム、離陸、着陸、GoTo、モード変更を行えます。

## 機能
- 機体状態のリアルタイム表示
- アーム / ディスアーム
- 離陸、着陸、GoTo
- `GUIDED` / `AUTO` / `RTL` / `LOITER` / `STABILIZE` のモード変更
- Leaflet 地図での機体位置表示と軌跡描画
- BlueOS Extension 用 `/register_service` 対応

## 今回の BlueOS 対応
- `MAV_ENDPOINT` 環境変数で MAVLink 接続先を切り替え可能
- 既定接続先を `udpout:host.docker.internal:14550` に変更
- WebSocket アプリ向けに `avoid_iframes: true` の `register_service` を実装
- `mode_mapping_byname` による明示モードマップを使用
- 自機以外の HEARTBEAT / telemetry を無視して状態の点滅を防止
- Leaflet 本体と画像アセットをローカル同梱
- BlueOS 用 `permissions` LABEL を含む Dockerfile を追加

## 動作確認済み
- ローカル Docker 起動
- BlueOS 実機へのインストール
- BlueOS 上での画面表示
- SITL 接続

## 前提条件
- Python 3.11 近辺での実行を想定
- ローカル確認では ArduPilot SITL が `tcp:127.0.0.1:5762` で待ち受けていること
- BlueOS では MAVLink Server が動作していること

## ローカル起動

### Python で起動
```bash
cd backend
pip install -r requirements.txt
python3 -m uvicorn main:app --host 0.0.0.0 --port 9999 --reload
```

ブラウザ:

```text
http://127.0.0.1:9999/
```

### Docker で起動
```bash
cd /home/ardupilot/GitHub/droneschool/workshop/21st/takashi-fujii/drone-web-app
docker build -t drone-web-app .
docker run --rm --network host -e MAV_ENDPOINT=tcp:127.0.0.1:5762 drone-web-app
```

ブラウザ:

```text
http://localhost:9999/
```

## BlueOS へのインストール方法

### 1. Docker Hub へ公開
```bash
cd /home/ardupilot/GitHub/droneschool/workshop/21st/takashi-fujii/drone-web-app
docker login
docker buildx create --use
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  --provenance=false \
  -t kod82fhi/drone-web-app:latest \
  --push .
```

Docker イメージ名:

```text
kod82fhi/drone-web-app:latest
```

### 2. BlueOS Extensions で登録
- `Create from scratch` を選ぶ
- Docker image に `kod82fhi/drone-web-app`
- Tag に `latest`
- JSON editor に Dockerfile の `permissions` LABEL と同じ値を入れる

`permissions`:

```json
{"ExposedPorts":{"9999/tcp":{}},"HostConfig":{"PortBindings":{"9999/tcp":[{"HostPort":"9999"}]},"ExtraHosts":["host.docker.internal:host-gateway"]}}
```

## 使い方
1. ブラウザでアプリを開く
2. `Connect` を押して機体へ接続する
3. 状態表示と地図が更新されることを確認する
4. `Arm`、`Takeoff`、`Go To`、`Land`、`Mode` を必要に応じて使う

## API
- `GET /`
  - フロント画面を返す
- `GET /register_service`
  - BlueOS Extension 用メタデータを返す
- `WS /ws`
  - 状態配信とコマンド送信用 WebSocket

## 補足
- 地図タイルは OpenStreetMap を利用しているため、完全オフライン時は地図画像が出ない場合があります
- Leaflet 本体は同梱済みのため、BlueOS ホットスポット環境でも `L is not defined` は発生しない構成です
- 追加アレンジとして、BlueOS Extension 化、Leaflet ローカル同梱、MAVLink 受信フィルタ、明示モードマップ対応を入れています
