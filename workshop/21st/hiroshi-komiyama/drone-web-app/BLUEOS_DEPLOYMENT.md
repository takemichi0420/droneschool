# BlueOS 上で動かす方法

## 前提条件
- BlueOS システムが動作している
- Docker Hub アカウント (または private registry)
- ローカル開発環境で `docker` がインストール済み

## デプロイ手順

### 1. Docker Hub にイメージをアップロード

```bash
# ローカルでビルド
docker build -t yourusername/drone-web-app:1.0.0 .
docker build -t yourusername/drone-web-app:latest .

# Docker Hub にログイン
docker login

# イメージをプッシュ
docker push yourusername/drone-web-app:1.0.0
docker push yourusername/drone-web-app:latest
```

### 2. BlueOS 管理画面からインストール

1. BlueOS の Web UI にアクセス: `http://blueos.local` または `http://<IP>`
2. 左パネルの **"Extensions"** をクリック
3. **"Install Extension"** ボタンをクリック
4. リポジトリ URL または イメージ名を入力:
   ```
   yourusername/drone-web-app:latest
   ```
5. **"Install"** をクリック

### 3. Extension が起動したか確認

- BlueOS 管理画面で `drone-web-app` がリストアップされていることを確認
- ステータスが **"Running"** になっているか確認

### 4. Web UI にアクセス

- BlueOS 内蔵ブラウザまたは外部ブラウザから以下にアクセス:
  ```
  http://blueos.local:9999
  または
  http://<BlueOS-IP>:9999
  ```

## ネットワーク接続

- **デフォルト**: `udpout:host.docker.internal:14550`
  - BlueOS の MAVLink Router を通じてドローンに接続
- **カスタム**: 環境変数 `MAV_ENDPOINT` で変更可能
  - 例: `tcp:192.168.1.100:5762` (別 PC の SITL)

## 設定

環境変数:
- `MAV_ENDPOINT`: MAVLink 接続文字列 (デフォルト: `udpout:host.docker.internal:14550`)

BlueOS Extension では、管理画面から環境変数を設定できます。

## トラブルシューティング

### Extension が起動しない
```bash
# BlueOS ホストで確認
docker ps | grep drone-web-app
docker logs <container_id>
```

### WebSocket が接続できない
- ファイアウォール設定を確認 (ポート 9999)
- BlueOS ネットワーク設定を確認

### MAVLink 接続できない
- `MAV_ENDPOINT` が正しいか確認
- BlueOS Router が起動しているか確認
- ネットワークの接続性を確認

## マルチアーキテクチャ対応 (github Actions を使用)

`.github/workflows/docker-publish.yml` を作成することで、自動ビルド・プッシュが可能です:
```yaml
name: Build and Push
on:
  push:
    branches: [main]
jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: docker/setup-buildx-action@v2
      - uses: docker/login-action@v2
        with:
          username: ${{ secrets.DOCKER_USERNAME }}
          password: ${{ secrets.DOCKER_PASSWORD }}
      - uses: docker/build-push-action@v4
        with:
          context: .
          platforms: linux/amd64,linux/arm64,linux/arm/v7
          push: true
          tags: yourusername/drone-web-app:latest
```

## 参考リンク
- [BlueOS Extensions](https://blueos.io/extensions)
- [Docker Documentation](https://docs.docker.com/)
- [BlueOS MAVLink Router](https://github.com/bluerobotics/mavlink-router)
