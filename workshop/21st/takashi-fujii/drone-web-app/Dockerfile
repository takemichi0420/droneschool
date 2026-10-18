FROM python:3.11-slim

LABEL version="1.0.0"
LABEL permissions="{\"ExposedPorts\":{\"9999/tcp\":{}},\"HostConfig\":{\"PortBindings\":{\"9999/tcp\":[{\"HostPort\":\"9999\"}]},\"ExtraHosts\":[\"host.docker.internal:host-gateway\"]}}"
LABEL authors="[{\"name\":\"Takashi Fujii\",\"email\":\"\"}]"
LABEL company="{\"name\":\"\",\"email\":\"\",\"about\":\"\"}"
LABEL type="other"
LABEL tags="[\"drone\",\"mavlink\"]"
LABEL readme=""
LABEL links="{}"
LABEL requirements=""

WORKDIR /app

COPY backend/requirements.txt ./backend/requirements.txt
RUN pip install --no-cache-dir -r backend/requirements.txt

COPY backend/ ./backend/
COPY frontend/ ./frontend/

WORKDIR /app/backend
EXPOSE 9999
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "9999", "--no-access-log"]
