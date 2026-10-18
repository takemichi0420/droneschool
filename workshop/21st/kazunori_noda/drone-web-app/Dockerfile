# drone-web-app/Dockerfile
# BlueOS Extension 対応版

FROM python:3.11-slim

# ---------------------------------------------------------------------------
# BlueOS Extension メタデータ
# permissions LABEL:
#   - NetworkMode bridge（デフォルト）
#   - HostPort 固定 9999（avoid_iframes で直接アクセスするため必須）
#   - ExtraHosts host.docker.internal → host-gateway
#     （コンテナ内から BlueOS Router の MAVLink に届けるため）
# ---------------------------------------------------------------------------
LABEL version="1.0.0"
LABEL permissions='\
{\
  "ExposedPorts": {\
    "9999/tcp": {}\
  },\
  "HostConfig": {\
    "ExtraHosts": ["host.docker.internal:host-gateway"],\
    "PortBindings": {\
      "9999/tcp": [{"HostIp": "0.0.0.0", "HostPort": "9999"}]\
    },\
    "NetworkMode": "bridge"\
  }\
}'
LABEL authors='[\
  {"name": "kazunorinoda", "email": ""}\
]'

WORKDIR /app


# 依存ライブラリ先にインストール（レイヤーキャッシュ活用）
COPY backend/requirements.txt ./requirements.txt
RUN pip install --no-cache-dir -r requirements.txt

# アプリ本体
COPY backend/ ./backend/
COPY frontend/ ./frontend/

# MAV_ENDPOINT は実行時に上書き可能
# 既定値: BlueOS Router への udpout 接続
ENV MAV_ENDPOINT="udpout:host.docker.internal:14550"

EXPOSE 9999

# --no-access-log: Helper の継続ヘルスチェックによるログ肥大を防ぐ
CMD ["python", "-m", "uvicorn", "backend.main:app",\
     "--host", "0.0.0.0", "--port", "9999", "--no-access-log"]