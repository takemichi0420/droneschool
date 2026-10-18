# Drone Web Controller

FastAPI + WebSocket + pymavlink を使った、ブラウザからドローンを操作する最小構成アプリです。

## 機能
- Connect / Arm / Disarm
- Takeoff / Land
- GoTo
- Mode change
- Forward / Back / Left / Right relative movement
- Real-time status panel
- Leaflet map with marker and flight track

## 前提条件
- Python 3.7+
- SITL or a real MAVLink vehicle listening at tcp:127.0.0.1:5762

## 起動手順
```bash
cd backend
pip install -r requirements.txt
uvicorn main:app --host 0.0.0.0 --port 9999 --reload
```

Then open:
```text
http://127.0.0.1:9999/
```

## 使い方
- Connect ボタンで MAVLink 接続を開始します。
- Arm / Takeoff / Land / GoTo / Mode で制御できます。
- Forward / Back / Left / Right は押下中のみ移動し、離すと停止します。

## BlueOS 上で実行

### Docker で実行
```bash
# ローカルテスト
docker build -t drone-web-app .
docker run --rm --network host -e MAV_ENDPOINT=tcp:127.0.0.1:5762 drone-web-app
```

### BlueOS Extension としてインストール

詳細は [BLUEOS_DEPLOYMENT.md](./BLUEOS_DEPLOYMENT.md) を参照してください。

クイックスタート:
```bash
# Docker Hub にプッシュ (yourusername は自分の Docker Hub ユーザー名に変更)
./deploy.sh yourusername 1.0.0

# BlueOS 管理画面から:
# Extensions → Install Extension → yourusername/drone-web-app:latest
```

その後、`http://blueos.local:9999` でアクセス可能

## 環境変数

- `MAV_ENDPOINT`: MAVLink 接続先 (デフォルト: `udpout:host.docker.internal:14550`)
  - 例: `tcp:127.0.0.1:5762` (ローカル SITL)
  - 例: `tcp:192.168.1.100:5762` (別 PC)
  - BlueOS で実行時は通常デフォルト値のままで動作

## ライセンス
MIT
