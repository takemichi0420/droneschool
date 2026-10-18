#!/usr/bin/env bash
# frontend/ ディレクトリで実行する
# 例: cd drone-web-app/frontend && bash ../templates/fetch-leaflet.sh

set -euo pipefail

LEAFLET_VERSION="1.9.4"
DEST="leaflet"

echo "=== Leaflet ${LEAFLET_VERSION} をローカルに同梱します ==="

mkdir -p "${DEST}/images"

BASE="https://unpkg.com/leaflet@${LEAFLET_VERSION}/dist"

curl -fsSL "${BASE}/leaflet.js"      -o "${DEST}/leaflet.js"
curl -fsSL "${BASE}/leaflet.css"     -o "${DEST}/leaflet.css"

# マーカー画像
for img in \
    marker-icon.png \
    marker-icon-2x.png \
    marker-shadow.png \
    layers.png \
    layers-2x.png; do
    curl -fsSL "${BASE}/images/${img}" -o "${DEST}/images/${img}"
done

echo "=== 完了: frontend/leaflet/ に配置されました ==="
echo ""
echo "index.html の CDN 参照を以下に差し替えてください:"
echo '  <link rel="stylesheet" href="/static/leaflet/leaflet.css" />'
echo '  <script src="/static/leaflet/leaflet.js"></script>'