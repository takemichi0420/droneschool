const TOKYO_STATION = [35.681236, 139.767125];

const map = L.map("map").setView(TOKYO_STATION, 16);
L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
  maxZoom: 19,
  attribution: "&copy; OpenStreetMap contributors",
}).addTo(map);

let droneMarker = null;
let trailLine = L.polyline([], { color: "#2563eb", weight: 3 }).addTo(map);

function updateMap(state) {
  if (state.latitude === 0 && state.longitude === 0) {
    // 緯度・経度がともに0は「まだ位置情報を受信していない」ことを示す初期値のため、
    // 実際のGPS取得前に軌跡・マーカーへ反映しない。
    return;
  }

  const latlng = [state.latitude, state.longitude];

  if (droneMarker === null) {
    droneMarker = L.marker(latlng).addTo(map);
  } else {
    droneMarker.setLatLng(latlng);
  }

  droneMarker
    .bindPopup(
      `緯度: ${state.latitude.toFixed(6)}<br>` +
        `経度: ${state.longitude.toFixed(6)}<br>` +
        `高度: ${state.altitude.toFixed(2)} m`
    );

  trailLine.addLatLng(latlng);
  map.panTo(latlng);
}

function clearTrail() {
  trailLine.setLatLngs([]);
}

function updateBatteryRow(battery) {
  const row = document.getElementById("row-battery");
  row.classList.remove("battery-warning", "battery-caution", "battery-critical", "battery-blink");

  if (battery <= 20) {
    row.classList.add("battery-critical", "battery-blink");
  } else if (battery <= 30) {
    row.classList.add("battery-critical");
  } else if (battery <= 40) {
    row.classList.add("battery-caution");
  } else if (battery <= 60) {
    row.classList.add("battery-warning");
  }
}

function updateStatusPanel(state) {
  document.getElementById("stat-connected").textContent = state.connected ? "接続済み" : "未接続";
  document.getElementById("stat-armed").textContent = state.armed ? "アーム" : "ディスアーム";
  document.getElementById("stat-mode").textContent = state.mode;
  document.getElementById("stat-latitude").textContent = state.latitude.toFixed(6);
  document.getElementById("stat-longitude").textContent = state.longitude.toFixed(6);
  document.getElementById("stat-altitude").textContent = state.altitude.toFixed(2);
  document.getElementById("stat-heading").textContent = Math.round(state.heading);
  document.getElementById("stat-battery").textContent = Math.round(state.battery);
  updateBatteryRow(state.battery);
}

function setLogMessage(message) {
  const el = document.getElementById("log-message");
  el.textContent = message;
}

function setWsIndicator(connected) {
  const el = document.getElementById("ws-indicator");
  if (connected) {
    el.textContent = "WS: 接続中";
    el.className = "badge badge-on";
  } else {
    el.textContent = "WS: 切断";
    el.className = "badge badge-off";
  }
}

let ws = null;

function connectWebSocket() {
  ws = new WebSocket(`ws://${window.location.host}/ws`);

  ws.onopen = () => {
    setWsIndicator(true);
    clearTrail();
  };

  ws.onmessage = (event) => {
    const msg = JSON.parse(event.data);
    if (msg.type === "state") {
      updateStatusPanel(msg.state);
      updateMap(msg.state);
    } else if (msg.type === "status") {
      setLogMessage(msg.message);
    }
  };

  ws.onclose = () => {
    setWsIndicator(false);
    setTimeout(connectWebSocket, 3000);
  };

  ws.onerror = () => {
    ws.close();
  };
}

function sendCommand(command) {
  if (ws === null || ws.readyState !== WebSocket.OPEN) {
    setLogMessage("WebSocketが未接続です");
    return;
  }
  ws.send(JSON.stringify(command));
}

document.getElementById("btn-connect").addEventListener("click", () => {
  sendCommand({ type: "connect" });
});

document.getElementById("btn-arm").addEventListener("click", () => {
  sendCommand({ type: "arm" });
});

document.getElementById("btn-disarm").addEventListener("click", () => {
  sendCommand({ type: "disarm" });
});

document.getElementById("btn-land").addEventListener("click", () => {
  sendCommand({ type: "land" });
});

document.getElementById("btn-takeoff").addEventListener("click", () => {
  const altitude = parseFloat(document.getElementById("input-takeoff-alt").value);
  sendCommand({ type: "takeoff", altitude });
});

document.getElementById("btn-goto").addEventListener("click", () => {
  const latitude = parseFloat(document.getElementById("input-goto-lat").value);
  const longitude = parseFloat(document.getElementById("input-goto-lon").value);
  const altitude = parseFloat(document.getElementById("input-goto-alt").value);
  sendCommand({ type: "goto", latitude, longitude, altitude });
});

document.getElementById("btn-set-mode").addEventListener("click", () => {
  const mode = document.getElementById("select-mode").value;
  sendCommand({ type: "mode", mode });
});

connectWebSocket();
