// Drone Web App フロントエンド

const DEFAULT_CENTER = [35.681236, 139.767125]; // 東京駅付近

let ws = null;
let map = null;
let marker = null;
let trackLine = null;

// ---------------------------------------------------------------- 地図

function initMap() {
  map = L.map('map').setView(DEFAULT_CENTER, 16);
  L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 19,
    attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
  }).addTo(map);

  marker = L.marker(DEFAULT_CENTER).addTo(map);
  marker.bindPopup('機体位置');

  trackLine = L.polyline([], { color: 'red', weight: 3 }).addTo(map);
}

// ---------------------------------------------------------------- WebSocket

function connectWebSocket() {
  ws = new WebSocket(`ws://${window.location.host}/ws`);

  ws.onopen = () => {
    trackLine.setLatLngs([]); // 再接続時に飛行軌跡をクリア
    showMessage('サーバーに接続しました');
  };

  ws.onmessage = (event) => {
    const msg = JSON.parse(event.data);
    if (msg.type === 'state') {
      updateStatus(msg.state);
    } else if (msg.type === 'status') {
      showMessage(msg.message);
    }
  };

  ws.onclose = () => {
    showMessage('サーバーと切断されました。3秒後に再接続します…');
    setTimeout(connectWebSocket, 3000);
  };
}

function sendCommand(payload) {
  if (!ws || ws.readyState !== WebSocket.OPEN) {
    showMessage('サーバーに未接続のため送信できません');
    return;
  }
  ws.send(JSON.stringify(payload));
}

// ---------------------------------------------------------------- 表示更新

function setText(id, text) {
  document.getElementById(id).textContent = text;
}

function showMessage(text) {
  setText('message', text);
}

function updateStatus(state) {
  setText('st-connected', state.connected ? '接続' : '未接続');
  setText('st-armed', state.armed ? 'ARM' : 'DISARM');
  setText('st-mode', state.mode);
  setText('st-lat', state.latitude.toFixed(6));
  setText('st-lon', state.longitude.toFixed(6));
  setText('st-alt', state.altitude.toFixed(2));
  setText('st-heading', String(Math.round(state.heading)));

  if (state.connected && (state.latitude !== 0 || state.longitude !== 0)) {
    const pos = [state.latitude, state.longitude];
    marker.setLatLng(pos);
    marker.setPopupContent(
      `緯度: ${state.latitude.toFixed(6)}<br>` +
      `経度: ${state.longitude.toFixed(6)}<br>` +
      `高度: ${state.altitude.toFixed(2)} m`
    );
    trackLine.addLatLng(pos);
    map.setView(pos); // 地図中心を追従
  }
}

// ---------------------------------------------------------------- ボタン

function setupControls() {
  document.getElementById('btn-connect').addEventListener('click', () => {
    sendCommand({ type: 'connect' });
  });

  document.getElementById('btn-arm').addEventListener('click', () => {
    sendCommand({ type: 'arm' });
  });

  document.getElementById('btn-disarm').addEventListener('click', () => {
    sendCommand({ type: 'disarm' });
  });

  document.getElementById('btn-takeoff').addEventListener('click', () => {
    const altitude = parseFloat(document.getElementById('takeoff-alt').value);
    if (isNaN(altitude) || altitude <= 0) {
      showMessage('離陸高度を正しく入力してください');
      return;
    }
    sendCommand({ type: 'takeoff', altitude });
  });

  document.getElementById('btn-land').addEventListener('click', () => {
    sendCommand({ type: 'land' });
  });

  document.getElementById('btn-goto').addEventListener('click', () => {
    const latitude = parseFloat(document.getElementById('goto-lat').value);
    const longitude = parseFloat(document.getElementById('goto-lon').value);
    const altitude = parseFloat(document.getElementById('goto-alt').value);
    if (isNaN(latitude) || isNaN(longitude) || isNaN(altitude)) {
      showMessage('GoTo の緯度・経度・高度を正しく入力してください');
      return;
    }
    sendCommand({ type: 'goto', latitude, longitude, altitude });
  });

  document.getElementById('btn-mode').addEventListener('click', () => {
    const mode = document.getElementById('mode-select').value;
    sendCommand({ type: 'mode', mode });
  });
}

// ---------------------------------------------------------------- 初期化

document.addEventListener('DOMContentLoaded', () => {
  initMap();
  setupControls();
  connectWebSocket(); // ページロード時に自動接続（機体へは繋がない）
});
