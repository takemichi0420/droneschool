const connectionStatus = document.getElementById('connectionStatus');
const armedStatus = document.getElementById('armedStatus');
const modeStatus = document.getElementById('modeStatus');
const latitudeStatus = document.getElementById('latitudeStatus');
const longitudeStatus = document.getElementById('longitudeStatus');
const altitudeStatus = document.getElementById('altitudeStatus');
// ★ 追加: バッテリー・GPS・RTK 表示要素
const batteryStatus = document.getElementById('batteryStatus');
const gpsFixStatus = document.getElementById('gpsFixStatus');

const connectBtn = document.getElementById('connectBtn');
const armBtn = document.getElementById('armBtn');
// ★ 追加: ディスアームボタン要素
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

// ★ 変更（前回）: 再接続タイマーを管理する変数
let reconnectTimer = null;

// ★ 追加: GPS Fix タイプを人が読める文字列に変換するマップ
const GPS_FIX_TYPES = {
    0: 'No GPS',
    1: 'No Fix',
    2: '2D Fix',
    3: '3D Fix',
    4: 'DGPS',
    5: 'RTK Float',
    6: 'RTK Fixed',
    7: 'Static',
    8: 'PPP',
};

function initMap() {
    map = L.map('map').setView([35.681236, 139.767125], 13);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
    }).addTo(map);
    droneMarker = L.marker([35.681236, 139.767125]).addTo(map)
        .bindPopup("Drone Position").openPopup();
    flightPathPolyline = L.polyline(flightPath, {color: 'red'}).addTo(map);
}

function connectWebSocket() {
    if (reconnectTimer) {
        clearTimeout(reconnectTimer);
        reconnectTimer = null;
    }

    ws = new WebSocket(`ws://${window.location.host}/ws`);

    ws.onopen = () => {
        connectionStatus.textContent = '接続済み';
        console.log('WebSocket connected');
        clearFlightPath();
        // ★ 変更（前回）: onopen での自動 connect 送信を削除
    };

    ws.onmessage = (event) => {
        let data;
        try {
            data = JSON.parse(event.data);
        } catch (e) {
            console.error('JSON パースエラー:', e);
            return;
        }

        if (data.type === 'status') {
            console.log('Backend Status:', data.message);

        } else if (data.type === 'state') {
            const s = data.state;

            armedStatus.textContent = s.armed ? 'アーム済み' : '未アーム';
            modeStatus.textContent = s.mode;
            latitudeStatus.textContent = s.latitude.toFixed(6);
            longitudeStatus.textContent = s.longitude.toFixed(6);
            altitudeStatus.textContent = s.altitude.toFixed(2);

            // ★ 追加: バッテリー残量の表示（null/undefined のときは "--" を表示）
            batteryStatus.textContent = (s.battery_remaining !== null && s.battery_remaining !== undefined)
                ? s.battery_remaining
                : '--';

            // ★ 追加: GPS Fix タイプの表示
            if (s.gps_fix_type !== null && s.gps_fix_type !== undefined) {
                gpsFixStatus.textContent = GPS_FIX_TYPES[s.gps_fix_type] ?? `Type ${s.gps_fix_type}`;
            } else {
                gpsFixStatus.textContent = '--';
            }

            // ★ 変更（前回）: 緯度・経度が 0,0 のとき地図を更新しない
            if (s.latitude !== 0 || s.longitude !== 0) {
                const newLatLng = new L.LatLng(s.latitude, s.longitude);
                droneMarker.setLatLng(newLatLng);
                droneMarker.setPopupContent(
                    `Drone Position<br>Lat: ${s.latitude.toFixed(6)}<br>` +
                    `Lon: ${s.longitude.toFixed(6)}<br>Alt: ${s.altitude.toFixed(2)}m`
                );
                map.panTo(newLatLng);

                flightPath.push(newLatLng);
                flightPathPolyline.setLatLngs(flightPath);
            }
        }
    };

    ws.onclose = () => {
        connectionStatus.textContent = '切断済み';
        console.log('WebSocket disconnected');
        clearFlightPath();
        reconnectTimer = setTimeout(connectWebSocket, 3000);
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

// ★ 追加: ディスアームボタンのイベントリスナー
disarmBtn.addEventListener('click', () => {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ type: 'disarm' }));
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
    window.confirm(latitude)
    window.confirm(longitude)
    window.confirm(altitude)
    if (!isNaN(latitude) && !isNaN(longitude) && !isNaN(altitude) && ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ type: 'goto', latitude: latitude, longitude: longitude, altitude: altitude }));
    }
});

setModeBtn.addEventListener('click', () => {
    const modeName = modeSelect.value;
    if (modeName && ws && ws.readyState === WebSocket.OPEN) {
        // ★ 変更（前回）: キー名を mode_name → mode に変更
        ws.send(JSON.stringify({ type: 'mode', mode: modeName }));
    }
});

document.addEventListener('DOMContentLoaded', () => {
    initMap();
    connectWebSocket();
});