let socket = null;
let map;
let marker;
let pathLine;
let pathPoints = [];
const DEFAULT_LAT = 35.681236;
const DEFAULT_LON = 139.767125;
const DEFAULT_ALT = 0.0;
const DEFAULT_POS = [DEFAULT_LAT, DEFAULT_LON];
let reconnectTimer = null;
let moveHoldTimer = null;
let activeMoveType = null;

function initMap() {
  map = L.map('map').setView(DEFAULT_POS, 16);
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    attribution: '&copy; OpenStreetMap contributors'
  }).addTo(map);
  marker = L.marker(DEFAULT_POS).addTo(map);
  marker.bindPopup('Waiting...');
  pathLine = L.polyline([], { weight: 3 }).addTo(map);
}

function connectWebSocket() {
  if (socket && socket.readyState === WebSocket.OPEN) {
    return;
  }
  socket = new WebSocket(`ws://${window.location.host}/ws`);
  socket.onopen = () => {
    setMessage('WebSocket connected');
  };
  socket.onmessage = (event) => {
    const msg = JSON.parse(event.data);
    if (msg.type === 'state') {
      updateState(msg.state);
    } else if (msg.type === 'status') {
      setMessage(msg.message);
    }
  };
  socket.onclose = () => {
    setMessage('Disconnected. reconnecting in 3 sec.');
    pathPoints = [];
    if (pathLine) {
      pathLine.setLatLngs([]);
    }
    if (reconnectTimer) {
      clearTimeout(reconnectTimer);
    }
    reconnectTimer = setTimeout(connectWebSocket, 3000);
  };
  socket.onerror = (err) => {
    console.error(err);
  };
}

function updateState(state) {
  const latitude = typeof state.latitude === 'number' ? state.latitude : DEFAULT_LAT;
  const longitude = typeof state.longitude === 'number' ? state.longitude : DEFAULT_LON;
  const altitude = typeof state.altitude === 'number' ? state.altitude : DEFAULT_ALT;

  document.getElementById('connected').textContent = String(state.connected);
  document.getElementById('armed').textContent = String(state.armed);
  document.getElementById('mode').textContent = state.mode;
  document.getElementById('latitude').textContent = latitude.toFixed(6);
  document.getElementById('longitude').textContent = longitude.toFixed(6);
  document.getElementById('altitude').textContent = altitude.toFixed(2);
  document.getElementById('heading').textContent = state.heading;
  updateMap({ ...state, latitude, longitude, altitude });
}

function updateMap(state) {
  const pos = [state.latitude, state.longitude];
  marker.setLatLng(pos);
  marker.setPopupContent(`Lat: ${state.latitude.toFixed(6)}<br>Lon: ${state.longitude.toFixed(6)}<br>Alt: ${state.altitude.toFixed(2)} m`);
  map.setView(pos);
  pathPoints.push(pos);
  pathLine.setLatLngs(pathPoints);
}

function setMessage(msg) {
  document.getElementById('message').textContent = msg;
}

function send(data) {
  if (!socket || socket.readyState !== WebSocket.OPEN) {
    setMessage('WebSocket not connected');
    return;
  }
  socket.send(JSON.stringify(data));
}

function setupButtons() {
  document.getElementById('connectBtn').onclick = () => send({ type: 'connect' });
  document.getElementById('armBtn').onclick = () => send({ type: 'arm' });
  document.getElementById('disarmBtn').onclick = () => send({ type: 'disarm' });
  document.getElementById('landBtn').onclick = () => send({ type: 'land' });
  document.getElementById('takeoffBtn').onclick = () => send({ type: 'takeoff', altitude: Number(document.getElementById('takeoffAltitude').value) });
  document.getElementById('gotoBtn').onclick = () => send({ type: 'goto', latitude: Number(document.getElementById('gotoLat').value), longitude: Number(document.getElementById('gotoLon').value), altitude: Number(document.getElementById('gotoAlt').value) });
  document.getElementById('modeBtn').onclick = () => send({ type: 'mode', mode: document.getElementById('modeSelect').value });

  const moveButtons = [
    ['forwardBtn', 'moveForward'],
    ['backBtn', 'moveBack'],
    ['leftBtn', 'moveLeft'],
    ['rightBtn', 'moveRight']
  ];
  moveButtons.forEach(([id, type]) => {
    const btn = document.getElementById(id);
    const start = (event) => {
      event.preventDefault();
      event.stopPropagation();
      activeMoveType = type;
      send({ type });
      if (moveHoldTimer) {
        clearInterval(moveHoldTimer);
      }
      moveHoldTimer = setInterval(() => {
        if (activeMoveType) {
          send({ type: activeMoveType });
        }
      }, 120);
    };
    const stop = (event) => {
      if (event) {
        event.preventDefault();
        event.stopPropagation();
      }
      if (moveHoldTimer) {
        clearInterval(moveHoldTimer);
        moveHoldTimer = null;
      }
      if (activeMoveType) {
        send({ type: 'moveStop' });
        activeMoveType = null;
      }
    };
    btn.addEventListener('pointerdown', start);
    btn.addEventListener('mousedown', start);
    btn.addEventListener('touchstart', start, { passive: false });
    btn.addEventListener('pointerup', stop);
    btn.addEventListener('pointerleave', stop);
    btn.addEventListener('pointercancel', stop);
    btn.addEventListener('mouseup', stop);
    btn.addEventListener('touchend', stop);
    btn.addEventListener('click', (event) => {
      event.preventDefault();
      event.stopPropagation();
    });
  });
}

window.onload = () => {
  initMap();
  setupButtons();
  connectWebSocket();
};
