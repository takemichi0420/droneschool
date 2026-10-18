Extension Identifier	kazinf.drone-web-app
Extension Name	Drone Web App
Docker image	kazinf/drone-web-app
Docker tag	latest


{
  "ExposedPorts": { "9999/tcp": {} },
  "HostConfig": {
    "PortBindings": { "9999/tcp": [{ "HostPort": "9999" }] },
    "ExtraHosts": ["host.docker.internal:host-gateway"]
  }
}