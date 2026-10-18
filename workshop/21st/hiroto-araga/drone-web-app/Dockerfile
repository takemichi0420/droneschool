FROM python:3.11-slim

# Create app directory mirroring original repo layout
WORKDIR /app/drone-web-app/backend

# Install runtime deps (requirements located in backend/)
COPY backend/requirements.txt /app/requirements.txt
RUN pip install --no-cache-dir -r /app/requirements.txt

# Copy backend and frontend into image preserving repo layout
COPY backend/ /app/drone-web-app/backend/
COPY frontend/ /app/drone-web-app/frontend/

# Expose the fixed port used by the app
ENV PORT=9999
EXPOSE 9999

# Docker LABEL for BlueOS permissions (bridge + HostPort fixed)
LABEL permissions='{"bridge": {"HostPort": "9999", "Protocol": "tcp"}, "ExtraHosts": ["host.docker.internal:host-gateway"]}'

CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "9999", "--no-access-log"]
