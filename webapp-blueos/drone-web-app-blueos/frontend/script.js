const connectionStatus = document.getElementById('connectionStatus');
const armedStatus = document.getElementById('armedStatus');
const modeStatus = document.getElementById('modeStatus');
const latitudeStatus = document.getElementById('latitudeStatus');
const longitudeStatus = document.getElementById('longitudeStatus');
const altitudeStatus = document.getElementById('altitudeStatus');

const connectBtn = document.getElementById('connectBtn');
const armBtn = document.getElementById('armBtn');
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
    ws = new WebSocket(`ws://${window.location.host}/ws`);

    ws.onopen = () => {
        connectionStatus.textContent = '接続済み';
        console.log('WebSocket connected');
        clearFlightPath(); // Clear previous flight path on new connection
        // Send a connect command to the backend to initiate drone connection
        ws.send(JSON.stringify({ type: 'connect' }));
    };

    ws.onmessage = (event) => {
        const data = JSON.parse(event.data);
        // console.log('Received:', data);

        if (data.type === 'status') {
            // This is a general status message from backend
            console.log('Backend Status:', data.message);
        } else {
            // This is drone telemetry data
            armedStatus.textContent = data.armed ? 'アーム済み' : '未アーム';
            modeStatus.textContent = data.mode;
            latitudeStatus.textContent = data.latitude.toFixed(6);
            longitudeStatus.textContent = data.longitude.toFixed(6);
            altitudeStatus.textContent = data.altitude.toFixed(2);

            // Update drone marker on map
            const newLatLng = new L.LatLng(data.latitude, data.longitude);
            droneMarker.setLatLng(newLatLng);
            droneMarker.setPopupContent(`Drone Position<br>Lat: ${data.latitude.toFixed(6)}<br>Lon: ${data.longitude.toFixed(6)}<br>Alt: ${data.altitude.toFixed(2)}m`).openPopup();
            map.panTo(newLatLng); // Center map on drone

            // Update flight path
            flightPath.push(newLatLng);
            flightPathPolyline.setLatLngs(flightPath);
        }
    };

    ws.onclose = () => {
        connectionStatus.textContent = '切断済み';
        console.log('WebSocket disconnected');
        setTimeout(connectWebSocket, 3000); // Attempt to reconnect after 3 seconds
    };

    ws.onerror = (err) => {
        console.error('WebSocket error:', err);
        connectionStatus.textContent = 'エラー';
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
        ws.send(JSON.stringify({ type: 'connect' }));
    } else {
        connectWebSocket();
    }
});

armBtn.addEventListener('click', () => {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ type: 'arm' }));
    }
});

takeoffBtn.addEventListener('click', () => {
    const altitude = parseFloat(takeoffAltitudeInput.value);
    if (!isNaN(altitude) && ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ type: 'takeoff', altitude: altitude }));
    }
});

landBtn.addEventListener('click', () => {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ type: 'land' }));
    }
});

gotoBtn.addEventListener('click', () => {
    const latitude = parseFloat(gotoLatitudeInput.value);
    const longitude = parseFloat(gotoLongitudeInput.value);
    const altitude = parseFloat(gotoAltitudeInput.value);
    if (!isNaN(latitude) && !isNaN(longitude) && !isNaN(altitude) && ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ type: 'goto', latitude: latitude, longitude: longitude, altitude: altitude }));
    }
});

setModeBtn.addEventListener('click', () => {
    const modeName = modeSelect.value;
    if (modeName && ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ type: 'mode', mode_name: modeName }));
    }
});

// Initialize on page load
document.addEventListener('DOMContentLoaded', () => {
    initMap();
    connectWebSocket();
});