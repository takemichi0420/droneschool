# Workshop 21st - Takashi Fujii

このワークフォルダには、提出対象アプリと参照用ファイルを配置しています。

## 提出対象
- [drone-web-app](/home/ardupilot/GitHub/droneschool/workshop/21st/takashi-fujii/drone-web-app)
  - FastAPI + WebSocket + pymavlink で実装したドローン Web アプリ
  - BlueOS Extension 化、Docker 化、Leaflet ローカル同梱、`/register_service` 対応済み

## 参照用
- [drone-web-app-blueos](/home/ardupilot/GitHub/droneschool/workshop/21st/takashi-fujii/drone-web-app-blueos)
  - 講座内で参照した完成版サンプル
- [build-prompt.md](/home/ardupilot/GitHub/droneschool/workshop/21st/takashi-fujii/build-prompt.md)
  - 元の Web アプリ生成時に使った補助プロンプト

## 動作確認
- ローカル Docker 起動を確認
- BlueOS 実機へのインストールを確認
- BlueOS 上での画面表示と SITL 接続を確認

## インストールと使用方法
- 提出対象アプリの詳細は [drone-web-app/README.md](/home/ardupilot/GitHub/droneschool/workshop/21st/takashi-fujii/drone-web-app/README.md) に記載
- Docker イメージ名は `kod82fhi/drone-web-app:latest`
- BlueOS へのインストール方法とローカル起動方法も同 README を参照
