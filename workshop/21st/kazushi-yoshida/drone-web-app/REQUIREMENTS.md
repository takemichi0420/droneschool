# 要件定義書 (REQUIREMENTS)

## 概要

既存の CLI ベースのドローン制御体験を、Web ブラウザから操作できるように置き換えるアプリケーション。
ローカルで動作し、FastAPI + WebSocket + pymavlink で MAVLink 機体(SITL)と通信する。

## 目的

- ブラウザから直感的にドローンへ接続し、状態をリアルタイムに確認しながら制御できるようにする。
- CLI 操作の学習コストを下げ、地図上で機体位置と軌跡を可視化する。

## ターゲットユーザー

- ドローン制御を学ぶ受講者・開発者
- ArduPilot SITL を用いてローカルで動作検証を行うエンジニア

## 機能要件

### 接続・制御コマンド(WebSocket 経由)

| コマンド | 挙動 | 入力 |
| --- | --- | --- |
| connect | 未接続なら機体への接続を開始 | — |
| arm | `MAV_CMD_COMPONENT_ARM_DISARM` param1=1 | — |
| disarm | `MAV_CMD_COMPONENT_ARM_DISARM` param1=0 | — |
| takeoff | `MAV_CMD_NAV_TAKEOFF`(param7=目標高度)。事前に GUIDED 切替 | 目標高度 |
| land | `MAV_CMD_NAV_LAND` | — |
| goto | `set_position_target_global_int_send`。事前に GUIDED 切替 | 緯度・経度・高度 |
| mode | `set_mode()` でモード変更 | モード名 |

- `takeoff` / `goto` は実行前に `GUIDED` への切替を試みる(最大5秒待機)。
- モード変更は `set_mode()` を使用し、`command_long` の `MAV_CMD_DO_SET_MODE` は使わない。

### リアルタイム状態

| 項目 | キー | 表示形式 |
| --- | --- | --- |
| 接続状態 | connected | — |
| アーム状態 | armed | — |
| フライトモード | mode | — |
| 緯度 | latitude | 小数6桁 |
| 経度 | longitude | 小数6桁 |
| 高度 | altitude | 小数2桁 |
| ヘディング | heading | 整数 |

- 初期値: 接続/アーム=false、モード=UNKNOWN、数値=0。
- `GLOBAL_POSITION_INT` から緯度・経度・高度・ヘディング、`HEARTBEAT` からアーム状態・モードを更新。

### 地図

- Leaflet + OpenStreetMap。初期中心は東京駅付近 `35.681236, 139.767125`。
- 機体位置マーカーを表示し、更新のたびに移動・中心追従。
- 飛行軌跡をポリラインで表示し、WebSocket 再接続時にクリア。
- マーカーのポップアップに緯度・経度・高度を表示。

## 非機能要件

- MAVLink 受信のブロッキング処理は executor に逃がし、WebSocket / イベントループを停止させない。
- 未接続時に制御コマンドを送っても致命的に落ちない。
- WebSocket 切断時は受信タスクを安全に停止し、MAVLink 受信の例外はログを残す。
- コマンド送信の成否は即断せず、状態更新はテレメトリー受信ベースで反映する(疎結合)。
- SITL の GCS など不正な HEARTBEAT 発生源を除外し、状態のチラつきを防ぐ。
- 起動ポートは 9999。接続先は既定で `tcp:127.0.0.1:5762`。
- スマートフォン幅では縦並びに崩れるレスポンシブ対応。

## 将来拡張案

- 複数機体の同時管理
- ミッション(ウェイポイント)の作成・アップロード
- バッテリー・GPS Fix・衛星数など詳細テレメトリーの表示
- 認証・アクセス制御(ローカル外公開時)
- 飛行ログの記録・再生
- HTTPS / WSS 対応
