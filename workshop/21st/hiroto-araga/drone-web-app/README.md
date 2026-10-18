# Drone Web Controller

ブラウザからドローンを操作するための最小構成 Web アプリです。

## 機能
- 接続状態の表示
- アーム / ディスアーム
- 離陸 / 着陸
- GoTo 送信
- モード変更
- リアルタイム位置の地図表示

## 技術スタック
- Python 3.7+
- FastAPI
- WebSocket
- pymavlink
- Leaflet

## 前提条件
- Python 3.7 以上
- MAVLink シミュレータまたは実機が `tcp:127.0.0.1:5762` で待ち受けていること

## 起動手順
```bash
cd backend
pip install -r requirements.txt
uvicorn main:app --port 9999 --reload
```

ブラウザで http://127.0.0.1:9999/ を開きます。

## 使い方
1. 「接続」ボタンで MAVLink 接続を開始します。
2. 状態が更新されたら各ボタンからコマンドを送信できます。
3. 地図上で機体の位置と軌跡を確認できます。
