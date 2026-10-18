# Drone Web App — BlueOS Extension

pymavlink + FastAPI + WebSocket によるドローン Web コントロールパネル。 <BR>
BlueOS Extension として動作し、ARM / TAKEOFF / LAND / GoTo などの操作と 
バッテリー残量・GNSS 状況のリアルタイム表示に対応します。

---

 

## 機能

 

- **リアルタイム状態表示**

  - 接続状態 / アーム状態 / フライトモード

  - 緯度・経度・高度
  
  - バッテリー残量（%）

  - GPSステータス


- **コマンド送信**

  - ARM / DISARM

  - TAKEOFF（高度指定）

  - LAND

  - GoTo（緯度・経度・高度指定）

  - フライトモード変更

- **地図表示**（Leaflet ローカル同梱・オフライン動作対応）

- **WebSocket** によるリアルタイム双方向通信

 

---

 

## 動作環境

 

| 項目 | 内容 |
|------|------|
| 対応機体 | ArduCopter（ArduPilot 4.x 以降） |
| BlueOS | 1.1.0 以降 |
| ポート | 9999/tcp |
| 通信方式 | MAVLink 2.0（UDP / TCP） |

 
---


## インストール方法（BlueOS Extension）

 

### Docker イメージ

kazunorinoda/drone-web-app:latest

 

 

### 手順

 

1. BlueOS Web UI（`http://192.168.42.1`）にアクセスします。

2. 左メニューから **Extensions → Extensions Manager** を開きます。

3. **Create from scratch** をクリックします。

4. 以下の通り入力します。

 

| フィールド | 入力値 |
|-----------|--------|
| Identifier | `kazunorinoda.drone-web-app` |
| Name | `Drone Web App` |
| Docker image | `kazunorinoda/drone-web-app` |
| Tag | `latest` |

 

5. **JSON エディタ**に以下の permissions を入力します。

 

```json
{
	"ExposedPorts": { "9999/tcp": {} },
    "HostConfig": {
        "PortBindings": { "9999/tcp": [{ "HostPort": "9999" }] },
        "ExtraHosts": ["host.docker.internal:host-gateway"]
    }
}
```
 
6. Install をクリックします。

7. 約30秒後、左メニューに Drone Web App が表示されます。

8. 使用方法

- アクセス

   左メニューの Drone Web App をクリックすると、
新しいウィンドウで http://192.168.42.1:9999 が開きます。

- 接続

  拡張起動時に MAVLink 接続を自動で開始します。<BR>
画面上部のステータスが 「MAVLink 接続成功」 になれば準備完了です。


## 技術スタック

| 領域 | 技術 |
|------|------|
| バックエンド | Python 3.7+ / FastAPI / uvicorn |
| MAVLink 通信 | pymavlink |
| リアルタイム通信 | WebSocket |
| フロントエンド | HTML / CSS / JavaScript（ビルドツール不要）|
| 地図 | Leaflet 1.9 + OpenStreetMap |


### SITL の例（ArduCopter)

```bash
sim_vehicle.py -v ArduCopter -L Kawachi --out udp:192.168.42.1:14551