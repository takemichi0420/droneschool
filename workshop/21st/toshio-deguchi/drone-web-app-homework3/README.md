# drone-web-app-homework3

## 概要

ドローンを Web ブラウザから操作できる最小構成のアプリケーションです。既存の CLI ベースのドローン制御体験を置き換え、ブラウザ上で MAVLink 機体への接続、状態のリアルタイム表示、各種コマンド送信（アーム／離陸／着陸／モード変更／指定座標への移動）ができます。

## 機能一覧

- 機体への MAVLink 接続（TCP、ボタン操作でのみ開始）
- アーム／ディスアーム
- 離陸（目標高度指定）／着陸
- 指定座標（緯度・経度・高度）への移動（GoTo）
- フライトモード変更（`GUIDED` / `AUTO` / `RTL` / `LOITER` / `STABILIZE`）
- 接続状態・アーム状態・フライトモード・緯度・経度・高度・ヘディング・バッテリー残量のリアルタイム表示
- バッテリー残量に応じたステータスパネルの背景色変化（60%以下: 黄色 / 40%以下: ピンク色 / 30%以下: 赤色 / 20%以下: 赤色点滅）
- Leaflet 地図上での機体位置マーカー表示と飛行軌跡の描画（WebSocket再接続時にクリアされ、位置未取得の初期値では描画しない）

## 技術スタック

- バックエンド: Python, FastAPI, WebSocket, pymavlink
- フロントエンド: 素の HTML / JavaScript / CSS（ビルドツール不要）
- 地図: Leaflet + OpenStreetMap タイル

## 前提条件

### ローカルでそのまま動かす場合

- Python 3.7 以上
- SITL（シミュレータ）が `tcp:127.0.0.1:5762` で待ち受けていること

### BlueOS Extension として動かす場合

- BlueOS が動作するコンパニオンコンピュータ（Raspberry Pi + Navigator など）が起動していること
- 機体（Pixhawk 等）が BlueOS に接続され、mavlink-router 経由でテレメトリが流れていること（標準構成であれば追加設定は不要です）
- ブラウザから BlueOS の Web UI（`http://<BlueOSのIP>/`）にアクセスできること

## 起動手順（ローカル開発）

接続先は環境変数 `MAV_ENDPOINT` で指定します（既定値は BlueOS Extension として動かす場合の `udpout:host.docker.internal:14550`）。ローカルの SITL に直接つなぐ場合は以下のように上書きしてください。

```bash
cd backend
pip install -r requirements.txt
MAV_ENDPOINT=tcp:127.0.0.1:5762 uvicorn main:app --port 9998 --reload
```

ブラウザで `http://127.0.0.1:9998/` を開きます。

## BlueOS Extension として使う

### Docker イメージ

このアプリの Docker イメージは Docker Hub に公開済みです。自分でビルドし直す必要はなく、以下の情報だけで誰でもインストールできます。

- イメージ: [`toshexit/drone-web-app-homework3`](https://hub.docker.com/r/toshexit/drone-web-app-homework3)（Public）
- タグ: `latest`

自分でビルド・pushし直したい場合は、リポジトリ直下で以下を実行します（Docker Hub へのログインが必要です）。

```bash
docker buildx build --platform linux/amd64,linux/arm64 --provenance=false \
  -t toshexit/drone-web-app-homework3:latest --push .
```

ローカルの Docker だけでビルド・動作確認したい場合（push はしない）:

```bash
docker build -t drone-web-app-hw3 .
docker run -d --rm -p 9998:9998 \
  -e MAV_ENDPOINT=udpout:host.docker.internal:14550 \
  --add-host=host.docker.internal:host-gateway \
  drone-web-app-hw3
```

### BlueOS へのインストール手順（Create from scratch）

1. BlueOS の Web UI を開き、左メニューの Extensions（パズルピースのアイコン）をクリックする
2. 「Installed」タブ右上の「+」→「Create from scratch」を選択する
3. 以下の内容を入力する

   | 項目 | 値 |
   |---|---|
   | Extension Identifier | `toshexit.drone-web-app-homework3` |
   | Extension Name | `Drone Web App HW3` |
   | Docker image | `toshexit/drone-web-app-homework3` |
   | Docker tag | `latest` |

4. permissions の JSON エディタ（初期値 `{}`）を以下の内容に書き換える。これは `Dockerfile` の `permissions` LABEL と同じ内容で、ポート 9998 の固定バインドと `host.docker.internal` 経由での MAVLink 接続を許可するものです。**この設定を省略・空のままにすると、ポートが割り当てられず左メニューに表示されません。**

   ```json
   {
     "ExposedPorts": { "9998/tcp": {} },
     "HostConfig": {
       "PortBindings": { "9998/tcp": [{ "HostPort": "9998" }] },
       "ExtraHosts": ["host.docker.internal:host-gateway"]
     }
   }
   ```

5. 「Create」を押すとイメージの pull とコンテナ起動が始まる（数十秒〜数分）
6. 完了すると BlueOS の左メニューにこの Extension のアイコンが表示される

### 起動確認（Definition of Done）

- [ ] 左メニューのアイコンをクリックすると、新しいウィンドウ（またはタブ）で `http://<BlueOSのIP>:9998/` が直接開く（WebSocket を使うため、BlueOS の画面埋め込みではなく別ウィンドウで開く仕様です）
- [ ] 「機体へ接続」を押すと数秒以内にステータスパネルの「接続状態」が「接続済み」になり、以後フリッカー（点滅・表示のバタつき）せず安定する
- [ ] 「アーム」を押すと「アーム状態」が「アーム」に変わる
- [ ] 離陸高度を指定して「離陸」を押すと、フライトモードが自動で `GUIDED` に切り替わり、高度が指定値に向けて上昇する
- [ ] 「着陸」を押すと高度が下降し、着地後は自動的にディスアームされる

うまく動かない場合は、BlueOS の Extensions 画面でこの Extension の「VIEW LOGS」を確認してください。`MAV_ENDPOINT` への接続がリトライを続けている場合は、機体側の MAVLink 出力設定（mavlink-router の UDP 14550 エンドポイント）を確認してください。

## 使い方

以下はローカル起動時の URL で説明しますが、BlueOS Extension として使う場合は `http://<BlueOSのIP>:9998/` に読み替えてください。

1. ブラウザで `http://127.0.0.1:9998/` を開く（この時点ではまだ機体には接続されません）
2. 「機体へ接続」ボタンを押して MAVLink 接続を開始する
3. 接続後、ステータスパネルに接続状態・アーム状態・フライトモード・位置情報・バッテリー残量がリアルタイムに表示される
4. バッテリー残量が 60%/40%/30% 以下になると、バッテリー残量の表示行の背景が黄色／ピンク色／赤色に変化し、20% 以下では赤色が点滅する
5. 「アーム」「ディスアーム」「着陸」ボタン、離陸高度を指定しての「離陸」、緯度・経度・高度を指定しての「GoTo実行」、モード選択＋「設定」で各コマンドを送信できる
6. 「離陸」ボタンの直後に「GoTo実行」を送っても、機体が実際に浮上するまでGoToの位置コマンドは送信を待機するため、離陸自体が中断されることはない
7. 地図上に機体の現在位置マーカーと飛行軌跡が表示され、WebSocket 再接続時に軌跡はクリアされる
