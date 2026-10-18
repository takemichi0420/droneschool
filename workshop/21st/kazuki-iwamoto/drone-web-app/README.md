# Drone Web App

ブラウザから MAVLink 対応ドローンへ接続し、状態確認と基本操作を行う最小構成の Web アプリケーションです。バックエンドは FastAPI、リアルタイム通信は WebSocket、MAVLink 通信は pymavlink、フロントエンドは素の HTML/CSS/JavaScript で実装しています。

## 機能一覧

- ブラウザから機体への MAVLink 接続を開始
- 接続状態、アーム状態、フライトモード、緯度、経度、高度、ヘディングをリアルタイム表示
- アーム、ディスアーム、離陸、着陸、GoTo、モード変更を Web UI から送信
- Leaflet + OpenStreetMap による現在位置マーカーと飛行軌跡の表示
- WebSocket 切断時の自動再接続
- モバイル幅に対応したレスポンシブレイアウト

## 技術スタック

- Python 3.7+
- FastAPI
- Uvicorn
- WebSocket
- pymavlink
- HTML / CSS / JavaScript
- Leaflet
- OpenStreetMap タイル

## 前提条件

- Python 3.7 以上
- SITL などのシミュレータまたは実機が `tcp:127.0.0.1:5762` で MAVLink 接続を待ち受けていること

## 起動手順

```bash
cd backend
pip install -r requirements.txt
uvicorn main:app --port 9999 --reload
```

ブラウザで次を開きます。

```text
http://127.0.0.1:9999/
```

## 使い方

1. ページを開くと WebSocket が自動接続されます。
2. `Connect` ボタンで MAVLink 接続を開始します。
3. ステータスパネルで接続状態、アーム状態、モード、位置、高度、ヘディングを確認します。
4. コントロールパネルから `Arm`、`Disarm`、`Takeoff`、`Land`、`GoTo`、`Set Mode` を実行します。
5. 地図上で機体位置と飛行軌跡を確認します。

`Takeoff` と `GoTo` は実行前に `GUIDED` モードへの切替を試みます。状態表示はコマンド送信結果ではなく、MAVLink テレメトリー受信に基づいて更新されます。
