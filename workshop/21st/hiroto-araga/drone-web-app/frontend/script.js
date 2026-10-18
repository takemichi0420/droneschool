let socket;
let map;
let marker;
let polyline;
let trail = [];

function updateStatus(state) {
  document.getElementById("connected").textContent = state.connected ? "true" : "false";
  document.getElementById("armed").textContent = state.armed ? "true" : "false";
  document.getElementById("mode").textContent = state.mode;
  document.getElementById("latitude").textContent = state.latitude.toFixed(6);
  document.getElementById("longitude").textContent = state.longitude.toFixed(6);
  document.getElementById("altitude").textContent = state.altitude.toFixed(2);
  document.getElementById("heading").textContent = state.heading;

  if (state.latitude && state.longitude) {
    const latlng = [state.latitude, state.longitude];
    if (!marker) {
      marker = L.marker(latlng).addTo(map);
    } else {
      marker.setLatLng(latlng);
    }
    marker.bindPopup(`Lat: ${state.latitude.toFixed(6)}<br>Lon: ${state.longitude.toFixed(6)}<br>Alt: ${state.altitude.toFixed(2)}`);
    map.panTo(latlng);

    trail.push(latlng);
    if (trail.length > 200) {
      trail.shift();
    }
    if (!polyline) {
      polyline = L.polyline(trail, { color: "#2563eb" }).addTo(map);
    } else {
      polyline.setLatLngs(trail);
    }
  }
}

function setStatusMessage(message) {
  document.getElementById("statusMessage").textContent = message;
}

function connectSocket() {
  if (socket && socket.readyState === WebSocket.OPEN) {
    return;
  }

  socket = new WebSocket(`ws://${window.location.host}/ws`);
  socket.onopen = () => setStatusMessage("接続済み");
  socket.onmessage = (event) => {
    const msg = JSON.parse(event.data);
    if (msg.type === "state") {
      updateStatus(msg.state);
    } else if (msg.type === "status") {
      setStatusMessage(msg.message);
    }
  };
  socket.onclose = () => {
    setStatusMessage("切断されました。3秒後に再接続します。")
    setTimeout(connectSocket, 3000);
    trail = [];
    if (polyline) {
      polyline.setLatLngs([]);
    }
  };
}

function sendCommand(payload) {
  if (socket && socket.readyState === WebSocket.OPEN) {
    socket.send(JSON.stringify(payload));
  } else {
    setStatusMessage("WebSocket未接続です");
  }
}

function initMap() {
  map = L.map("map").setView([35.681236, 139.767125], 13);
  L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
    maxZoom: 19,
    attribution: "&copy; OpenStreetMap contributors"
  }).addTo(map);
}

function bindControls() {
  document.getElementById("connectButton").addEventListener("click", () => sendCommand({ type: "connect" }));
  document.getElementById("armButton").addEventListener("click", () => sendCommand({ type: "arm" }));
  document.getElementById("disarmButton").addEventListener("click", () => sendCommand({ type: "disarm" }));
  document.getElementById("landButton").addEventListener("click", () => sendCommand({ type: "land" }));
  document.getElementById("takeoffButton").addEventListener("click", () => {
    const altitude = Number(document.getElementById("takeoffHeight").value || 0);
    sendCommand({ type: "takeoff", altitude });
  });
  document.getElementById("gotoButton").addEventListener("click", () => {
    const latitude = Number(document.getElementById("gotoLat").value || 0);
    const longitude = Number(document.getElementById("gotoLon").value || 0);
    const altitude = Number(document.getElementById("gotoAlt").value || 0);
    sendCommand({ type: "goto", latitude, longitude, altitude });
  });
  document.getElementById("modeButton").addEventListener("click", () => {
    const mode = document.getElementById("modeSelect").value;
    sendCommand({ type: "mode", mode });
  });
}

window.addEventListener("DOMContentLoaded", () => {
  initMap();
  bindControls();
  connectSocket();
});
