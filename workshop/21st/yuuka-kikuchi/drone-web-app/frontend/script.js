const connectionStatus = document.getElementById('connectionStatus');
const armedStatus = document.getElementById('armedStatus');
const modeStatus = document.getElementById('modeStatus');
const latitudeStatus = document.getElementById('latitudeStatus');
const longitudeStatus = document.getElementById('longitudeStatus');
const altitudeStatus = document.getElementById('altitudeStatus');
const headingStatus = document.getElementById('headingStatus');

const connectBtn = document.getElementById('connectBtn');
const armBtn = document.getElementById('armBtn');
const disarmBtn = document.getElementById('disarmBtn');
const takeoffBtn = document.getElementById('takeoffBtn');
const landBtn = document.getElementById('landBtn');
const gotoBtn = document.getElementById('gotoBtn');
const setModeBtn = document.getElementById('setModeBtn');

const takeoffAltitudeInput = document.getElementById('takeoffAltitude');
const gotoLatitudeInput = document.getElementById('gotoLatitude');
const gotoLongitudeInput = document.getElementById('gotoLongitude');
const gotoAltitudeInput = document.getElementById('gotoAltitude');
const modeSelect = document.getElementById('modeSelect');

let ws;
let map;
let droneMarker;
let flightPath = [];
let flightPathPolyline;
let reconnectAttempts = 0;
const MAX_RECONNECT_ATTEMPTS = 10;

function log(msg) {
    console.log(`[${new Date().toLocaleTimeString()}] ${msg}`);
}

// Initialize Leaflet Map
function initMap() {
    map = L.map('map').setView([35.681236, 139.767125], 13); // Default to Tokyo
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
    }).addTo(map);
    droneMarker = L.marker([35.681236, 139.767125]).addTo(map)
        .bindPopup("Drone Position").openPopup();
    flightPathPolyline = L.polyline(flightPath, {color: 'red'}).addTo(map);
}

function connectWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws`;
    
    log(`Connecting to WebSocket at ${wsUrl}...`);
    connectionStatus.textContent = '接続中...';
    ws = new WebSocket(wsUrl);

    ws.onopen = () => {
        log('✓ WebSocket connected to server');
        reconnectAttempts = 0;
        clearFlightPath(); // Clear previous flight path on new connection
    };

    ws.onmessage = (event) => {
        const data = JSON.parse(event.data);

        if (data.type === 'status') {
            // Status message from backend
            log(`Status: ${data.message}`);
            console.log(`Backend message: ${data.message}`);
        } else if (data.type === 'state') {
            // Drone telemetry data
            const state = data.state;
            const wasConnected = connectionStatus.textContent.includes('接続');
            const isConnected = state.connected;
            
            connectionStatus.textContent = isConnected ? '✓ 接続済み' : '✗ 未接続';
            if (isConnected && !wasConnected) {
                log('✓ Drone connected!');
            }
            
            armedStatus.textContent = state.armed ? '✓ アーム済み' : '未アーム';
            modeStatus.textContent = state.mode;
            latitudeStatus.textContent = state.latitude.toFixed(6);
            longitudeStatus.textContent = state.longitude.toFixed(6);
            altitudeStatus.textContent = state.altitude.toFixed(2);
            headingStatus.textContent = Math.round(state.heading);

            // Only update map if we have valid coordinates
            if (state.latitude !== 0 || state.longitude !== 0) {
                const newLatLng = new L.LatLng(state.latitude, state.longitude);
                droneMarker.setLatLng(newLatLng);
                droneMarker.setPopupContent(
                    `Drone Position<br>Lat: ${state.latitude.toFixed(6)}<br>Lon: ${state.longitude.toFixed(6)}<br>Alt: ${state.altitude.toFixed(2)}m`
                ).openPopup();
                map.panTo(newLatLng);

                // Update flight path
                flightPath.push(newLatLng);
                flightPathPolyline.setLatLngs(flightPath);
            }
        }
    };

    ws.onclose = () => {
        log('✗ WebSocket disconnected');
        connectionStatus.textContent = '切断済み';
        reconnectAttempts++;
        if (reconnectAttempts <= MAX_RECONNECT_ATTEMPTS) {
            const delay = Math.min(3000 * reconnectAttempts, 30000);
            log(`Reconnecting in ${delay/1000}s... (attempt ${reconnectAttempts}/${MAX_RECONNECT_ATTEMPTS})`);
            setTimeout(connectWebSocket, delay);
        } else {
            log('✗ Max reconnection attempts reached');
            connectionStatus.textContent = 'エラー（再接続失敗）';
        }
    };

    ws.onerror = (err) => {
        log(`✗ WebSocket error: ${err}`);
        connectionStatus.textContent = 'エラー';
        console.error('WebSocket error:', err);
    };
}

function clearFlightPath() {
    flightPath = [];
    if (flightPathPolyline) {
        flightPathPolyline.setLatLngs(flightPath);
    }
}

// --- Event Listeners for Commands ---
connectBtn.addEventListener('click', () => {
    if (ws && ws.readyState === WebSocket.OPEN) {
        log('Sending connect command to drone...');
        ws.send(JSON.stringify({ type: 'connect' }));
    } else {
        log('WebSocket not ready, attempting to reconnect...');
        connectWebSocket();
    }
});

armBtn.addEventListener('click', () => {
    if (ws && ws.readyState === WebSocket.OPEN) {
        log('Arming drone...');
        ws.send(JSON.stringify({ type: 'arm' }));
    } else {
        log('WebSocket not open');
    }
});

disarmBtn.addEventListener('click', () => {
    if (ws && ws.readyState === WebSocket.OPEN) {
        log('Disarming drone...');
        ws.send(JSON.stringify({ type: 'disarm' }));
    } else {
        log('WebSocket not open');
    }
});

takeoffBtn.addEventListener('click', () => {
    const altitude = parseFloat(takeoffAltitudeInput.value);
    if (!isNaN(altitude) && ws && ws.readyState === WebSocket.OPEN) {
        log(`Takeoff to ${altitude}m...`);
        ws.send(JSON.stringify({ type: 'takeoff', altitude: altitude }));
    } else {
        log('Invalid altitude or WebSocket not open');
    }
});

landBtn.addEventListener('click', () => {
    if (ws && ws.readyState === WebSocket.OPEN) {
        log('Landing drone...');
        ws.send(JSON.stringify({ type: 'land' }));
    } else {
        log('WebSocket not open');
    }
});

gotoBtn.addEventListener('click', () => {
    const latitude = parseFloat(gotoLatitudeInput.value);
    const longitude = parseFloat(gotoLongitudeInput.value);
    const altitude = parseFloat(gotoAltitudeInput.value);
    if (!isNaN(latitude) && !isNaN(longitude) && !isNaN(altitude) && ws && ws.readyState === WebSocket.OPEN) {
        log(`GoTo [${latitude}, ${longitude}, ${altitude}m]...`);
        ws.send(JSON.stringify({ type: 'goto', latitude: latitude, longitude: longitude, altitude: altitude }));
    } else {
        log('Invalid coordinates or WebSocket not open');
    }
});

setModeBtn.addEventListener('click', () => {
    const modeName = modeSelect.value;
    if (modeName && ws && ws.readyState === WebSocket.OPEN) {
        log(`Setting mode to ${modeName}...`);
        ws.send(JSON.stringify({ type: 'mode', mode_name: modeName }));
    } else {
        log('Invalid mode or WebSocket not open');
    }
});

// Initialize on page load
document.addEventListener('DOMContentLoaded', () => {
    log('Page loaded, initializing...');
    initMap();
    connectWebSocket();
});