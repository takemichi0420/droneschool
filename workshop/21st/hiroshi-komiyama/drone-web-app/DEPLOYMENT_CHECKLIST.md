# BlueOS Deployment Checklist

## 📋 Pre-Deployment Verification

- [x] Docker image builds successfully
- [x] HEALTHCHECK configured (HTTP GET on port 9999)
- [x] BlueOS permissions labels configured
  - [x] PortBindings/9999 fixed port
  - [x] ExtraHosts host.docker.internal access
- [x] WebSocket endpoint functional (/ws)
- [x] Service registration endpoint (/register_service)
- [x] Environment variables properly documented
- [x] All commands tested (arm, disarm, mode, land, takeoff, goto)
- [x] Frontend UI responsive and working
- [x] Leaflet map bundled locally (no CDN dependencies)

## 🚀 Deployment Steps

### Step 1: Prepare Docker Hub Account
```bash
# Login to Docker Hub
docker login

# Verify credentials stored
cat ~/.docker/config.json | head -20
```
- [ ] Docker Hub account active
- [ ] Credentials configured

### Step 2: Push to Docker Hub
```bash
# Using deploy script (recommended)
./deploy.sh yourusername 1.0.0

# OR manual method
docker build -t yourusername/drone-web-app:1.0.0 .
docker push yourusername/drone-web-app:1.0.0
docker tag yourusername/drone-web-app:1.0.0 yourusername/drone-web-app:latest
docker push yourusername/drone-web-app:latest
```
- [ ] Image built locally
- [ ] Image pushed to Docker Hub
- [ ] Image is public (verify at hub.docker.com)

### Step 3: Install on BlueOS
```
1. Go to: http://blueos.local (or http://<blueos-ip>)
2. Navigate to: Extensions → Install Extension
3. Enter image: yourusername/drone-web-app:latest
4. Click: Install
5. Wait for: Status → Running (green)
```
- [ ] Extension installed successfully
- [ ] Extension status is "Running"
- [ ] No error messages in extension logs

### Step 4: Verify Installation
```bash
# Check if service is accessible
curl -v http://blueos.local:9999/

# Open in browser
http://blueos.local:9999
```
- [ ] HTTP port 9999 responds
- [ ] Web UI loads and displays
- [ ] WebSocket connection establishes
- [ ] Mode selector dropdown populated

### Step 5: Verify MAVLink Connection
```bash
# In browser console or WebSocket client
# Click "Connect" button and verify:
- [ ] "Status: Connected" displays
- [ ] Vehicle telemetry updates (lat/lon/altitude)
- [ ] Map updates with vehicle position
- [ ] Mode shows correct value (e.g., "GUIDED")
```

### Step 6: Test All Commands
- [ ] Arm button → vehicle armed
- [ ] Disarm button → vehicle disarmed
- [ ] Takeoff button → vehicle takes off
- [ ] Land button → vehicle descends
- [ ] GoTo button → vehicle navigates
- [ ] Mode selector → mode changes applied
- [ ] Movement controls → vehicle responds
- [ ] Real-time telemetry updates

## 🔍 Troubleshooting

### Extension shows "Unhealthy"
- Check BlueOS extension logs
- Verify docker image has HEALTHCHECK configured
- Ensure port 9999 is accessible

### Cannot connect to vehicle
- Verify MAV_ENDPOINT environment variable is correct
  - Default: `udpout:host.docker.internal:14550` (BlueOS Router)
- Check BlueOS Router is running and listening
- Verify network connectivity between container and router

### WebSocket connection fails
- Check browser console for connection errors
- Verify http://blueos.local:9999 is accessible
- Check firewall rules for port 9999

### Mode selector shows empty
- Verify vehicle HEARTBEAT received
- Check backend logs for MODE_MAP initialization
- Ensure autopilot type is supported (Copter/Plane)

## 📊 Performance Notes

- Container startup: ~5-10 seconds
- HEALTHCHECK interval: 10 seconds
- WebSocket ping/pong: Automatic (browser WebSocket API)
- Telemetry update rate: 2-5 Hz (from SITL/vehicle)

## 🔐 Security Considerations

- Extension runs on BlueOS local network only
- No authentication required (typical for local tools)
- WebSocket communication unencrypted (localhost only)
- To expose externally: Use reverse proxy with HTTPS/TLS

## 📝 Maintenance

### Update to new version
```bash
# Build and push new version
./deploy.sh yourusername 1.1.0

# On BlueOS: Reinstall extension with new version tag
Extensions → Reinstall → yourusername/drone-web-app:1.1.0
```

### View extension logs
```
BlueOS Web UI → Extensions → Select drone-web-app → View Logs
```

### Reset to defaults
```
Extensions → drone-web-app → Remove
# Reinstall from Docker Hub
```

---

**Status: READY FOR DEPLOYMENT** ✅
