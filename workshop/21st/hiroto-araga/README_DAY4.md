使用方法
1. 接続
2．GUIDEDモードに設定
3．アーム
4．離陸
5．GoTo
6．着陸

インストール方法
Extension Identifier: yuhonium.drone-web-app
Extension Name: Drone Web App
Docker Image: yuhonium/drone-web-app
Docker tag: latest
JSON Editor:
{
  "ExposedPorts": {
    "9999/tcp": {}
  },
  "HostConfig": {
    "PortBindings": {
      "9999/tcp": [
        {
          "HostPort": "9999"
        }
      ]
    },
    "ExtraHosts": [
      "host.docker.internal:host-gateway"
    ]
  }
}