# dronify-blueos Dockerfile テンプレート
# 置換: <PORT>（既定9999）, <Your Name>/<email>。HostPort は方式で選ぶ（下記コメント参照）。
FROM python:3.11-slim

LABEL version="1.0.0"
# WebSocket + avoid_iframes（直接ポートを開く）→ HostPort を固定: "<PORT>"
# HTTP ポーリング + 埋め込み（works_in_relative_paths）→ HostPort は自動: ""
LABEL permissions='{"ExposedPorts":{"<PORT>/tcp":{}},"HostConfig":{"PortBindings":{"<PORT>/tcp":[{"HostPort":"<PORT>"}]},"ExtraHosts":["host.docker.internal:host-gateway"]}}'
LABEL authors='[{"name":"<Your Name>","email":"<you@example.com>"}]'
LABEL company='{"name":"","email":"","about":""}'
LABEL type="other"
LABEL tags='["drone","mavlink"]'
LABEL readme=''
LABEL links='{}'
LABEL requirements=''

WORKDIR /app

# 依存だけ先に入れてキャッシュを効かせる（パスはアプリ構成に合わせて調整）
COPY backend/requirements.txt ./backend/requirements.txt
RUN pip install --no-cache-dir -r backend/requirements.txt

COPY backend/ ./backend/
COPY frontend/ ./frontend/

WORKDIR /app/backend
EXPOSE <PORT>
# --no-access-log: Helper の継続ヘルスチェックによるログ肥大（VIEW LOGS タイムアウト）を防ぐ
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "<PORT>", "--no-access-log"]
