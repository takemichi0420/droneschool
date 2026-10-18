FROM python:3.11-slim

LABEL version="1.0.0"
# WebSocket + avoid_iframes（直接ポートを開く）のため HostPort は固定
LABEL permissions='{"ExposedPorts":{"9998/tcp":{}},"HostConfig":{"PortBindings":{"9998/tcp":[{"HostPort":"9998"}]},"ExtraHosts":["host.docker.internal:host-gateway"]}}'
LABEL authors='[{"name":"toshio-deguchi","email":"lifeliteracyclub@yahoo.co.jp"}]'
LABEL company='{"name":"","email":"","about":""}'
LABEL type="other"
LABEL tags='["drone","mavlink"]'
LABEL readme=''
LABEL links='{}'
LABEL requirements=''

WORKDIR /app

COPY backend/requirements.txt ./backend/requirements.txt
RUN pip install --no-cache-dir -r backend/requirements.txt

COPY backend/ ./backend/
COPY frontend/ ./frontend/

WORKDIR /app/backend
EXPOSE 9998
# --no-access-log: Helper の継続ヘルスチェックによるログ肥大（VIEW LOGS タイムアウト）を防ぐ
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "9998", "--no-access-log"]
