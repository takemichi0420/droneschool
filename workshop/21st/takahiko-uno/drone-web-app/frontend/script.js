"use strict";

// ---------------------------------------------------------------------------
// 効果音（Web Audio API。音声ファイル不要）
//   - 受付OK  : 「ピッ」 高め単発
//   - 受付不可: 「ピピ」 低め2連
//   - 飛行中  : 「プッ」 1秒ごと
// ブラウザはユーザー操作前に音を鳴らせないため、クリック契機で有効化する。
// ---------------------------------------------------------------------------
let audioCtx = null;

function ensureAudio() {
  if (audioCtx === null) {
    const AC = window.AudioContext || window.webkitAudioContext;
    if (!AC) return null;
    audioCtx = new AC();
  }
  if (audioCtx.state === "suspended") audioCtx.resume();
  return audioCtx;
}

// 単発ビープ。freq(Hz), 長さ(ms), 波形/音量/開始遅延(ms)。
function tone(freq, durMs, { type = "square", volume = 0.15, delayMs = 0 } = {}) {
  const ctx = ensureAudio();
  if (!ctx) return;
  const start = ctx.currentTime + delayMs / 1000;
  const end = start + durMs / 1000;
  const osc = ctx.createOscillator();
  const gain = ctx.createGain();
  osc.type = type;
  osc.frequency.value = freq;
  // プチッというクリックノイズを避けるため軽くフェード。
  gain.gain.setValueAtTime(0, start);
  gain.gain.linearRampToValueAtTime(volume, start + 0.005);
  gain.gain.setValueAtTime(volume, Math.max(start + 0.005, end - 0.01));
  gain.gain.linearRampToValueAtTime(0, end);
  osc.connect(gain);
  gain.connect(ctx.destination);
  osc.start(start);
  osc.stop(end + 0.02);
}

function beepAccept() {
  tone(1046, 90, { volume: 0.18 }); // ピッ
}
function beepReject() {
  tone(440, 90, { volume: 0.18 }); // ピ
  tone(440, 90, { volume: 0.18, delayMs: 130 }); // ピ
}
function beepFly() {
  tone(660, 70, { type: "sine", volume: 0.12 }); // プッ
}

// ---------------------------------------------------------------------------
// 地図（Leaflet）
// ---------------------------------------------------------------------------
const TOKYO = [35.681236, 139.767125];

let map = null;
let marker = null;
let trackLine = null;
let gotoMarker = null; // クリックで置く GoTo 目標マーカー

// 地図の初期化。Leaflet の読み込み失敗などで例外が起きても、
// WebSocket 接続（下部の connectWs）まで巻き込まないよう独立させる。
function initMap() {
  if (typeof L === "undefined") {
    console.error("Leaflet が読み込めていません。地図は無効化されます。");
    return;
  }
  // マーカー画像をローカル同梱パスに固定（既定は CDN 相対で解決に失敗しやすい）。
  L.Icon.Default.imagePath = "/static/vendor/leaflet/images/";

  map = L.map("map").setView(TOKYO, 16);
  L.tileLayer("https://tile.openstreetmap.org/{z}/{x}/{y}.png", {
    maxZoom: 19,
    attribution: "&copy; OpenStreetMap contributors",
  }).addTo(map);

  trackLine = L.polyline([], { color: "#2b7cff", weight: 3 }).addTo(map);

  // 地図クリックで GoTo 目標の緯度・経度を入力欄へ反映する。
  map.on("click", (e) => setGotoTarget(e.latlng.lat, e.latlng.lng));

  // flex レイアウト内で初期サイズを取り違えて白抜けするのを防ぐ。
  setTimeout(() => map.invalidateSize(), 0);
}

// GoTo 目標を入力欄へ反映し、地図上に目標マーカーを表示する。
function setGotoTarget(lat, lon) {
  document.getElementById("in-goto-lat").value = lat.toFixed(6);
  document.getElementById("in-goto-lon").value = lon.toFixed(6);

  if (map) {
    const pos = [lat, lon];
    if (gotoMarker === null) {
      // 機体マーカーと区別できるよう半透明の円マーカーにする。
      gotoMarker = L.circleMarker(pos, {
        radius: 8,
        color: "#e67e22",
        weight: 2,
        fillColor: "#e67e22",
        fillOpacity: 0.6,
      }).addTo(map);
    } else {
      gotoMarker.setLatLng(pos);
    }
    gotoMarker.bindTooltip("GoTo 目標").openTooltip();
  }

  log(`GoTo 目標を設定: ${lat.toFixed(6)}, ${lon.toFixed(6)}`);
}

function clearTrack() {
  if (trackLine) trackLine.setLatLngs([]);
}

function updateMap(state) {
  if (!map) return; // 地図が無効なら何もしない
  const { latitude, longitude, altitude } = state;
  // 初期値 (0,0) は無効値として扱い、実データが来るまで描画しない。
  if (!latitude && !longitude) return;

  const pos = [latitude, longitude];
  const popup = `緯度: ${latitude.toFixed(6)}<br>経度: ${longitude.toFixed(
    6
  )}<br>高度: ${altitude.toFixed(2)} m`;

  if (marker === null) {
    marker = L.marker(pos).addTo(map);
  } else {
    marker.setLatLng(pos);
  }
  marker.bindPopup(popup);

  trackLine.addLatLng(pos);
  map.setView(pos); // 位置更新のたびに地図中心を追従
}

// ---------------------------------------------------------------------------
// ステータス表示
// ---------------------------------------------------------------------------
function updateStatus(state) {
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
}

function log(message) {
  const el = document.getElementById("log");
  el.textContent = message;
}

// ---------------------------------------------------------------------------
// WebSocket クライアント
// ---------------------------------------------------------------------------
let ws = null;
let latestState = null; // 直近に受信した機体状態（飛行中判定に使う）

function setIndicator(connected) {
  const el = document.getElementById("ws-indicator");
  el.textContent = connected ? "サーバー接続中" : "サーバー未接続";
  el.className = "badge " + (connected ? "connected" : "disconnected");
}

function connectWs() {
  ws = new WebSocket(`ws://${window.location.host}/ws`);

  ws.onopen = () => {
    setIndicator(true);
    clearTrack(); // 再接続時に飛行軌跡をクリア
  };

  ws.onmessage = (event) => {
    let msg;
    try {
      msg = JSON.parse(event.data);
    } catch (e) {
      return;
    }
    if (msg.type === "state") {
      latestState = msg.state;
      updateStatus(msg.state);
      updateMap(msg.state);
    } else if (msg.type === "status") {
      log(msg.message);
      // バックエンドの受付結果に応じて鳴らす。
      if (msg.ok === true) beepAccept();
      else if (msg.ok === false) beepReject();
    }
  };

  ws.onclose = () => {
    setIndicator(false);
    // 切断時は 3 秒後に再接続
    setTimeout(connectWs, 3000);
  };

  ws.onerror = () => {
    ws.close();
  };
}

function send(command) {
  // クリック（ユーザー操作）契機で音声を有効化する。
  ensureAudio();
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(command));
    // 送信成功時の「ピッ/ピピ」は、成否が確定するバックエンドの status で鳴らす。
  } else {
    log("サーバーに接続されていません");
    beepReject();
  }
}

// ---------------------------------------------------------------------------
// ボタン操作
// ---------------------------------------------------------------------------
// 要素が見つからなくても全体が止まらないよう、個別に安全登録するヘルパ。
function on(id, handler) {
  const el = document.getElementById(id);
  if (!el) {
    console.error(`要素が見つかりません: #${id}（ボタン操作が無効になります）`);
    return;
  }
  el.addEventListener("click", handler);
}

function wireButtons() {
  on("btn-connect", () => send({ type: "connect" }));
  on("btn-arm", () => send({ type: "arm" }));
  on("btn-disarm", () => send({ type: "disarm" }));
  on("btn-land", () => send({ type: "land" }));

  on("btn-takeoff", () => {
    const alt = parseFloat(document.getElementById("in-takeoff-alt").value);
    send({ type: "takeoff", altitude: alt });
  });

  on("btn-goto", () => {
    const lat = parseFloat(document.getElementById("in-goto-lat").value);
    const lon = parseFloat(document.getElementById("in-goto-lon").value);
    const alt = parseFloat(document.getElementById("in-goto-alt").value);
    if (Number.isNaN(lat) || Number.isNaN(lon)) {
      log("緯度・経度を入力してください");
      beepReject();
      return;
    }
    send({ type: "goto", latitude: lat, longitude: lon, altitude: alt });
  });

  on("btn-mode", () => {
    const mode = document.getElementById("sel-mode").value;
    send({ type: "mode", mode: mode });
  });
}

// DOM がまだ準備中でも確実に登録されるようにする。
if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", wireButtons);
} else {
  wireButtons();
}

// 地図初期化は WebSocket 接続と切り離す。地図が失敗しても接続は必ず試みる。
try {
  initMap();
} catch (e) {
  console.error("地図の初期化に失敗しました:", e);
}

// ページロード時に自動接続
connectWs();

// 離陸中・飛行中（アーム済み かつ 高度 > 0.5m）は 1 秒ごとに「プッ」。
setInterval(() => {
  if (!latestState) return;
  const flying = latestState.armed && latestState.altitude > 0.5;
  if (flying) beepFly();
}, 1000);

// タブ復帰・リサイズ時に地図が白抜けするのを防ぐ。
window.addEventListener("resize", () => {
  if (map) map.invalidateSize();
});
