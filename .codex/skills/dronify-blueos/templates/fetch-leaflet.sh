#!/usr/bin/env bash
# Leaflet をローカル同梱する（CDN 依存を排除し、オフライン/ホットスポットでも地図を表示）。
# 使い方: frontend/ ディレクトリで実行する。
#   bash fetch-leaflet.sh [version]   # 既定 1.7.1
set -euo pipefail

VER="${1:-1.7.1}"
BASE="https://unpkg.com/leaflet@${VER}/dist"

mkdir -p leaflet/images
curl -sSL -o leaflet/leaflet.js  "${BASE}/leaflet.js"
curl -sSL -o leaflet/leaflet.css "${BASE}/leaflet.css"
for img in marker-icon.png marker-icon-2x.png marker-shadow.png layers.png layers-2x.png; do
  curl -sSL -o "leaflet/images/${img}" "${BASE}/images/${img}"
done

echo "Leaflet ${VER} を frontend/leaflet/ に同梱しました。"
echo "index.html の参照を以下に差し替えてください（WebSocket+avoid_iframes は絶対パスでOK）:"
echo '  <link rel="stylesheet" href="/static/leaflet/leaflet.css" />'
echo '  <script src="/static/leaflet/leaflet.js"></script>'
echo "HTTP ポーリング+埋め込みの場合は先頭の / を外して相対パス（static/leaflet/...）にする。"
