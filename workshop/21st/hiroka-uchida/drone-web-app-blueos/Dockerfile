# dronify-blueos Dockerfile (generated)
# PORT=9999
FROM python:3.11-slim

LABEL version="1.0.0"
# WebSocket + avoid_iframes（直接ポートを開く）→ HostPort を固定: "9999"
LABEL permissions='{"ExposedPorts":{"9999/tcp":{}},"HostConfig":{"PortBindings":{"9999/tcp":[{"HostPort":"9999"}]},"ExtraHosts":["host.docker.internal:host-gateway"]}}'
LABEL authors='[{"name":"fujihide77","email":""}]'
LABEL company='{"name":"","email":"","about":""}'
LABEL type="other"
LABEL tags='["drone","mavlink"]'
LABEL readme=''
LABEL links='{}'
LABEL requirements=''

WORKDIR /app

# Install dependencies (adjust path if needed)
COPY backend/requirements.txt ./backend/requirements.txt
RUN pip install --no-cache-dir -r backend/requirements.txt

COPY backend/ ./backend/
COPY frontend/ ./frontend/

WORKDIR /app/backend
EXPOSE 9999
# --no-access-log to avoid log noise
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "9999", "--no-access-log"]
