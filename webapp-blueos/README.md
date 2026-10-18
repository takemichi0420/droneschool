# Vibe コーディング — ドローン Web 制御アプリ

生成 AI でドローンの Web 制御アプリを作り、**BlueOS Extension** に仕上げるための教材一式です。素版と BlueOS 版の2つの実物と、生成用プロンプトを収録しています。

## 構成

```text
vibe-coding/
├── build-prompt.md          # 素版を生成するプロンプト
├── drone-web-app/           # 素版（ローカル SITL で動く最小構成）
└── drone-web-app-blueos/    # BlueOS 版（Extension 化まで適用済み）
```

- **build-prompt.md** … 仕様と技術スタック（FastAPI / pymavlink / WebSocket / Leaflet）を与えて素版を生成させるプロンプト。
- **drone-web-app/** … プロンプトから生成した最小構成。接続・ARM・離陸・着陸・GoTo・モード変更と、状態／現在位置のリアルタイム表示。接続先は `tcp:127.0.0.1:5762`。各フォルダに `README.md`（使い方）と `REQUIREMENTS.md`（仕様）を同梱。
- **drone-web-app-blueos/** … 素版に BlueOS Extension の要件を適用したもの。

## 素版 → BlueOS 版の差分

2つのフォルダの差分が、そのまま「BlueOS 化で必要な対応」です（`diff -r drone-web-app drone-web-app-blueos`）。

| 変更 | 内容 |
| --- | --- |
| `+ Dockerfile` | `permissions` などの LABEL（bridge + ポート固定） |
| `+ .dockerignore` | イメージ最小化 |
| `+ frontend/leaflet/` | Leaflet をローカル同梱（オフライン対策） |
| `~ backend/main.py` | `register_service`・`host.docker.internal` 接続・機体タイプ明示のモードマップ・受信フィルタ・接続先の環境変数化 |
| `~ frontend/index.html` | Leaflet 参照を CDN からローカルへ変更 |

> 手順は BlueOS アプリ開発ガイドを参照してください。
