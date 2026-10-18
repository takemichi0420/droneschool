const TOKYO_STATION = [35.681236, 139.767125];

let socket = null;
let reconnectTimer = null;
let marker = null;
let pathLine = null;
let pathPoints = [];

const map = L.map("map").setView(TOKYO_STATION, 16);
L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
  maxZoom: 19,
  attribution: "&copy; OpenStreetMap contributors",
}).addTo(map);

marker = L.marker(TOKYO_STATION).addTo(map);
pathLine = L.polyline([], { color: "#2563eb", weight: 4 }).addTo(map);

function connectWebSocket() {
  clearTimeout(reconnectTimer);
  socket = new WebSocket(`ws://${window.location.host}/ws`);

  socket.addEventListener("open", () => {
    setWebSocketStatus("connected");
    clearFlightPath();
  });

  socket.addEventListener("message", (event) => {
    const msg = JSON.parse(event.data);
    if (msg.type === "state") {
      updateState(msg.state);
    } else if (msg.type === "status") {
      setMessage(msg.message);
    }
  });

  socket.addEventListener("close", () => {
    setWebSocketStatus("disconnected");
    reconnectTimer = setTimeout(connectWebSocket, 3000);
  });

  socket.addEventListener("error", () => {
    setWebSocketStatus("error");
    socket.close();
  });
}

function sendCommand(command) {
  if (!socket || socket.readyState !== WebSocket.OPEN) {
    setMessage("WebSocket is not connected.");
    return;
  }
  socket.send(JSON.stringify(command));
}

function updateState(state) {
  document.getElementById("state-connected").textContent = state.connected;
  document.getElementById("state-armed").textContent = state.armed;
  document.getElementById("state-mode").textContent = state.mode;
  document.getElementById("state-latitude").textContent = Number(state.latitude).toFixed(6);
  document.getElementById("state-longitude").textContent = Number(state.longitude).toFixed(6);
  document.getElementById("state-altitude").textContent = `${Number(state.altitude).toFixed(2)} m`;
  document.getElementById("state-heading").textContent = `${Math.round(Number(state.heading))} deg`;

  if (Number(state.latitude) !== 0 || Number(state.longitude) !== 0) {
    updateMap(state);
  }
}

function updateMap(state) {
  const position = [Number(state.latitude), Number(state.longitude)];
  marker.setLatLng(position);
  marker.bindPopup(
    `Lat: ${Number(state.latitude).toFixed(6)}<br>` +
    `Lon: ${Number(state.longitude).toFixed(6)}<br>` +
    `Alt: ${Number(state.altitude).toFixed(2)} m`
  );
  pathPoints.push(position);
  pathLine.setLatLngs(pathPoints);
  map.setView(position, map.getZoom());
}

function clearFlightPath() {
  pathPoints = [];
  pathLine.setLatLngs(pathPoints);
}

function setWebSocketStatus(status) {
  document.getElementById("ws-status").textContent = `WebSocket: ${status}`;
}

function setMessage(message) {
  document.getElementById("message-log").textContent = message;
}

document.getElementById("connect-btn").addEventListener("click", () => {
  sendCommand({ type: "connect" });
});

document.getElementById("arm-btn").addEventListener("click", () => {
  sendCommand({ type: "arm" });
});

document.getElementById("disarm-btn").addEventListener("click", () => {
  sendCommand({ type: "disarm" });
});

document.getElementById("land-btn").addEventListener("click", () => {
  sendCommand({ type: "land" });
});

document.getElementById("takeoff-btn").addEventListener("click", () => {
  sendCommand({
    type: "takeoff",
    altitude: Number(document.getElementById("takeoff-altitude").value),
  });
});

document.getElementById("goto-btn").addEventListener("click", () => {
  sendCommand({
    type: "goto",
    latitude: Number(document.getElementById("goto-latitude").value),
    longitude: Number(document.getElementById("goto-longitude").value),
    altitude: Number(document.getElementById("goto-altitude").value),
  });
});

document.getElementById("mode-btn").addEventListener("click", () => {
  sendCommand({
    type: "mode",
    mode: document.getElementById("mode-select").value,
  });
});

connectWebSocket();
