FROM python:3.11-slim

WORKDIR /app

# Install build dependencies for PyMAVLink on ARM architectures
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements and install dependencies
COPY backend/requirements.txt .
RUN pip install --no-cache-dir --default-timeout=100 -r requirements.txt

# Copy application code and frontend
COPY backend/main.py .
COPY frontend frontend/

# Expose port 9999 for BlueOS
EXPOSE 9999

# BlueOS Extension Labels
#AIが出力した状態
#LABEL permissions='[{"name":"PortBindings/9999","description":"Fixed port for WebSocket","input":"PortBindings/9999","value":"9999"},{"name":"ExtraHosts","description":"Access host.docker.internal","input":"ExtraHosts","value":"host.docker.internal:host-gateway"}]'
#LABEL org.opencontainers.image.source="https://github.com/yourusername/drone-web-app"
#LABEL org.opencontainers.image.description="Drone Web Controller - FastAPI WebSocket Interface"
#LABEL org.opencontainers.image.version="1.0.0"
LABEL version="1.0.0"
# WebSocket + avoid_iframes（直接ポートを開く）→ HostPort を固定: "9999"
LABEL permissions='{"ExposedPorts":{"9999/tcp":{}},"HostConfig":{"PortBindings":{"9999/tcp":[{"HostPort":"9999"}]},"ExtraHosts":["host.docker.internal:host-gateway"]}}'
LABEL authors='[{"name":"comy33","email":""}]'
LABEL company='{"name":"","email":"","about":""}'
LABEL type="other"
LABEL tags='["drone","mavlink"]'
LABEL readme=''
LABEL links='{}'
LABEL requirements=''

LABEL org.opencontainers.image.source="https://github.com/comy33/drone-web-app"
LABEL org.opencontainers.image.description="Drone Web Controller - FastAPI WebSocket Interface"
LABEL org.opencontainers.image.version="1.0.0"


# Health check (BlueOS uses this to verify service is healthy)
HEALTHCHECK --interval=10s --timeout=5s --start-period=30s --retries=3 \
    CMD curl -f http://localhost:9999/ || exit 1

# Run uvicorn with no access logging (prevent log bloat from Helper health checks)
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "9999", "--no-access-log"]
