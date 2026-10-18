Hello ArduPilot! 21st

宿題３ Drone Web App について
詳細な手順は、講習テキスト通りですが drone-web-app フォルダ内に格納しています。

以下にインストールのための最低限の情報を記載します
BlueOS の Extensions メニュー
   → Installed 
   → 「+」 →(Create from scratch) から以下を入力:

項目	                値
Extension Identifier	tuno68k.drone-web-app
Extension Name        Drone Web App
Docker image	        tuno68k/drone-web-app
Docker tag            latest


## テキスト通りで変更ありません ##
Custom settings (Permissions JSON):
{
  "ExposedPorts": { "9999/tcp": {} },
  "HostConfig": {
    "PortBindings": { "9999/tcp": [{ "HostPort": "9999" }] },
    "ExtraHosts": ["host.docker.internal:host-gateway"]
  }
}


成果物一覧
• Docker イメージ(公開済み): tuno68k/drone-web-app:latest
• リポジトリ: tuno68k/drone-web-app(status: active)
                https://hub.docker.com/r/tuno68k/drone-web-app/tags
• タグ: latest のみ
• 公開設定: パブリック公開(is_private: false)  Docker Hub ログイン不要で pull 可能。
• マルチアーキ対応: linux/amd64 と linux/arm64 の両方あり(OCI image index)
• ソースコード: workshop/21st/takahiko-uno/drone-web-app/(ブランチ 21st_takahiko-uno)
