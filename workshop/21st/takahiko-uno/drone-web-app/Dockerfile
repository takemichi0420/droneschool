# Drone Web App — BlueOS Extension
# WebSocket + avoid_iframes 方式のため HostPort を 9999 に固定する。
FROM python:3.11-slim

LABEL version="1.0.0"
# permissions: bridge + PortBindings(HostPort 固定) + host.docker.internal 解決。
# WebSocket + avoid_iframes（直接ポートを開く）なので HostPort は固定する。
LABEL permissions='{"ExposedPorts":{"9999/tcp":{}},"HostConfig":{"PortBindings":{"9999/tcp":[{"HostPort":"9999"}]},"ExtraHosts":["host.docker.internal:host-gateway"]}}'
LABEL authors='[{"name":"Takahiko UNO","email":"tnx68k@gmail.com"}]'
LABEL company='{"name":"","email":"","about":""}'
LABEL type="other"
LABEL tags='["drone","mavlink"]'
LABEL readme=''
LABEL links='{}'
LABEL requirements=''

WORKDIR /app

# 依存だけ先に入れてキャッシュを効かせる。
COPY backend/requirements.txt ./backend/requirements.txt
RUN pip install --no-cache-dir -r backend/requirements.txt

COPY backend/ ./backend/
COPY frontend/ ./frontend/

WORKDIR /app/backend
EXPOSE 9999
# --no-access-log: Helper の継続ヘルスチェックによるログ肥大（VIEW LOGS タイムアウト）を防ぐ。
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "9999", "--no-access-log"]
