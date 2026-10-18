"use strict";

// ---- 地図 -----------------------------------------------------------------

const TOKYO = [35.681236, 139.767125]; // 東京駅付近

const map = L.map("map").setView(TOKYO, 16);
L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
  maxZoom: 19,
  attribution: "&copy; OpenStreetMap contributors",
}).addTo(map);

// 新ウィンドウ表示(avoid_iframes)やレイアウト確定前に初期化されると
// サイズが 0 のままで地図が描画されないことがある。確定後に再計算する。
// (BlueOS ホットスポット等オフライン時はタイルが灰色になるが仕様範囲)
function refreshMapSize() {
  map.invalidateSize();
}
window.addEventListener("load", () => setTimeout(refreshMapSize, 200));
window.addEventListener("resize", refreshMapSize);
setTimeout(refreshMapSize, 500);

let marker = null;
let trackLine = L.polyline([], { color: "#e2492d", weight: 3 }).addTo(map);
let trackPoints = [];

function updateMap(state) {
  const lat = state.latitude;
  const lon = state.longitude;
  if (!lat && !lon) return; // 未取得(0,0)は無視

  const pos = [lat, lon];
  const popup =
    `緯度: ${lat.toFixed(6)}<br>` +
    `経度: ${lon.toFixed(6)}<br>` +
    `高度: ${state.altitude.toFixed(2)} m`;

  if (marker === null) {
    marker = L.marker(pos).addTo(map);
  } else {
    marker.setLatLng(pos);
  }
  marker.bindPopup(popup);

  // 軌跡を追加し中心を追従
  trackPoints.push(pos);
  trackLine.setLatLngs(trackPoints);
  map.setView(pos);
}

function clearTrack() {
  trackPoints = [];
  trackLine.setLatLngs([]);
}

// ---- ステータス表示 -------------------------------------------------------

function renderState(state) {
  document.getElementById("st-connected").textContent = state.connected
    ? "接続中"
    : "未接続";
  document.getElementById("st-armed").textContent = state.armed
    ? "ARMED"
    : "DISARMED";
  document.getElementById("st-mode").textContent = state.mode;
  document.getElementById("st-lat").textContent = state.latitude.toFixed(6);
  document.getElementById("st-lon").textContent = state.longitude.toFixed(6);
  document.getElementById("st-alt").textContent = state.altitude.toFixed(2);
  document.getElementById("st-hdg").textContent = Math.round(state.heading);

  updateMap(state);
}

// ---- ログ -----------------------------------------------------------------

function log(message) {
  const el = document.getElementById("log");
  const line = document.createElement("div");
  line.textContent = message;
  el.prepend(line);
  while (el.childElementCount > 30) {
    el.removeChild(el.lastChild);
  }
}

// ---- WebSocket ------------------------------------------------------------

let ws = null;

function setWsIndicator(connected) {
  const el = document.getElementById("ws-indicator");
  if (connected) {
    el.textContent = "サーバー接続中";
    el.classList.remove("ws-off");
    el.classList.add("ws-on");
  } else {
    el.textContent = "サーバー未接続";
    el.classList.remove("ws-on");
    el.classList.add("ws-off");
  }
}

function connectWs() {
  ws = new WebSocket(`ws://${window.location.host}/ws`);

  ws.onopen = () => {
    setWsIndicator(true);
    clearTrack(); // 再接続時に軌跡クリア
    log("サーバーへ接続しました");
  };

  ws.onmessage = (event) => {
    const msg = JSON.parse(event.data);
    if (msg.type === "state") {
      renderState(msg.state);
    } else if (msg.type === "status") {
      log(msg.message);
    }
  };

  ws.onclose = () => {
    setWsIndicator(false);
    log("サーバーとの接続が切れました。3秒後に再接続します");
    setTimeout(connectWs, 3000);
  };

  ws.onerror = () => {
    log("WebSocket エラーが発生しました");
  };
}

function send(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(obj));
  } else {
    log("サーバー未接続のため送信できません");
  }
}

// ---- ボタン操作 -----------------------------------------------------------

document.getElementById("btn-connect").addEventListener("click", () => {
  send({ type: "connect" });
});
document.getElementById("btn-arm").addEventListener("click", () => {
  send({ type: "arm" });
});
document.getElementById("btn-disarm").addEventListener("click", () => {
  send({ type: "disarm" });
});
document.getElementById("btn-land").addEventListener("click", () => {
  send({ type: "land" });
});
document.getElementById("btn-takeoff").addEventListener("click", () => {
  const altitude = parseFloat(
    document.getElementById("in-takeoff-alt").value
  );
  send({ type: "takeoff", altitude });
});
document.getElementById("btn-goto").addEventListener("click", () => {
  const latitude = parseFloat(document.getElementById("in-goto-lat").value);
  const longitude = parseFloat(document.getElementById("in-goto-lon").value);
  const altitude = parseFloat(document.getElementById("in-goto-alt").value);
  if (isNaN(latitude) || isNaN(longitude) || isNaN(altitude)) {
    log("GoTo の緯度・経度・高度を入力してください");
    return;
  }
  send({ type: "goto", latitude, longitude, altitude });
});
document.getElementById("btn-mode").addEventListener("click", () => {
  const mode = document.getElementById("sel-mode").value;
  send({ type: "mode", mode });
});

// ---- 起動時に自動接続 ------------------------------------------------------

connectWs();
