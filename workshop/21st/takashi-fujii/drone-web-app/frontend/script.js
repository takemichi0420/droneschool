const wsProtocol = window.location.protocol === 'https:' ? 'wss' : 'ws';
const wsUrl = `${wsProtocol}://${window.location.host}/ws`;
let ws = null;

// Map setup
const map = L.map('map').setView([35.681236, 139.767125], 18);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 22,
    attribution: '© OpenStreetMap contributors'
}).addTo(map);

const droneIcon = L.icon({
    iconUrl: '/static/leaflet/images/marker-icon-2x.png',
    shadowUrl: '/static/leaflet/images/marker-shadow.png',
    iconSize: [25, 41],
    iconAnchor: [12, 41],
    popupAnchor: [1, -34],
    shadowSize: [41, 41]
});

let marker = null;
let pathPolyline = null;
let pathCoordinates = [];

// DOM Elements
const els = {
    connected: document.getElementById('val-connected'),
    armed: document.getElementById('val-armed'),
    mode: document.getElementById('val-mode'),
    lat: document.getElementById('val-lat'),
    lon: document.getElementById('val-lon'),
    alt: document.getElementById('val-alt'),
    hdg: document.getElementById('val-hdg'),
    sysMsg: document.getElementById('system-messages'),
    
    btnConnect: document.getElementById('btn-connect'),
    btnArm: document.getElementById('btn-arm'),
    btnDisarm: document.getElementById('btn-disarm'),
    btnTakeoff: document.getElementById('btn-takeoff'),
    btnLand: document.getElementById('btn-land'),
    btnGoto: document.getElementById('btn-goto'),
    btnMode: document.getElementById('btn-mode'),
    
    inTakeoffAlt: document.getElementById('input-takeoff-alt'),
    inGotoLat: document.getElementById('input-goto-lat'),
    inGotoLon: document.getElementById('input-goto-lon'),
    inGotoAlt: document.getElementById('input-goto-alt'),
    selMode: document.getElementById('select-mode')
};

function connectWebSocket() {
    ws = new WebSocket(wsUrl);
    
    ws.onopen = () => {
        logMessage("WebSocket connected to server");
        pathCoordinates = [];
        if (pathPolyline) {
            map.removeLayer(pathPolyline);
            pathPolyline = null;
        }
    };
    
    ws.onmessage = (event) => {
        const data = JSON.parse(event.data);
        if (data.type === 'state') {
            updateState(data.state);
        } else if (data.type === 'status') {
            logMessage(data.message);
        }
    };
    
    ws.onclose = () => {
        logMessage("WebSocket disconnected. Reconnecting in 3s...");
        setTimeout(connectWebSocket, 3000);
    };
    
    ws.onerror = (error) => {
        console.error("WebSocket Error:", error);
    };
}

function updateState(state) {
    els.connected.textContent = state.connected ? "True" : "False";
    els.connected.style.color = state.connected ? "var(--success)" : "inherit";
    
    els.armed.textContent = state.armed ? "True" : "False";
    els.armed.style.color = state.armed ? "var(--danger)" : "inherit";
    
    els.mode.textContent = state.mode;
    
    els.lat.textContent = state.latitude.toFixed(6);
    els.lon.textContent = state.longitude.toFixed(6);
    els.alt.textContent = state.altitude.toFixed(2) + " m";
    els.hdg.textContent = state.heading + "°";
    
    // Update map if we have valid coordinates
    if (state.latitude !== 0 || state.longitude !== 0) {
        const latlng = [state.latitude, state.longitude];
        
        if (!marker) {
            marker = L.marker(latlng, {icon: droneIcon}).addTo(map);
            map.setView(latlng, 18);
        } else {
            marker.setLatLng(latlng);
        }
        
        marker.bindPopup(`Lat: ${state.latitude.toFixed(6)}<br>Lon: ${state.longitude.toFixed(6)}<br>Alt: ${state.altitude.toFixed(2)}m`).update();
        
        // Update polyline
        pathCoordinates.push(latlng);
        if (!pathPolyline) {
            pathPolyline = L.polyline(pathCoordinates, {color: 'red'}).addTo(map);
        } else {
            pathPolyline.setLatLngs(pathCoordinates);
        }
    }
}

function logMessage(msg) {
    els.sysMsg.textContent = msg;
    els.sysMsg.style.backgroundColor = 'rgba(59, 130, 246, 0.3)';
    setTimeout(() => {
        els.sysMsg.style.backgroundColor = 'rgba(15, 23, 42, 0.8)';
    }, 300);
}

function sendCommand(cmd) {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify(cmd));
    } else {
        logMessage("Error: WebSocket not connected");
    }
}

// Event Listeners
els.btnConnect.addEventListener('click', () => sendCommand({type: 'connect'}));
els.btnArm.addEventListener('click', () => sendCommand({type: 'arm'}));
els.btnDisarm.addEventListener('click', () => sendCommand({type: 'disarm'}));

els.btnTakeoff.addEventListener('click', () => {
    sendCommand({
        type: 'takeoff',
        altitude: parseFloat(els.inTakeoffAlt.value) || 10
    });
});

els.btnLand.addEventListener('click', () => sendCommand({type: 'land'}));

els.btnGoto.addEventListener('click', () => {
    sendCommand({
        type: 'goto',
        latitude: parseFloat(els.inGotoLat.value) || 0,
        longitude: parseFloat(els.inGotoLon.value) || 0,
        altitude: parseFloat(els.inGotoAlt.value) || 0
    });
});

els.btnMode.addEventListener('click', () => {
    sendCommand({
        type: 'mode',
        mode: els.selMode.value
    });
});

// Initialize
connectWebSocket();
