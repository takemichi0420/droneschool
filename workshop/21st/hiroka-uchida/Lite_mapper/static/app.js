const API = {
    connect: "/connect",
    disconnect: "/disconnect",
    status: "/telemetry",
    mode: "/mode",
    arm: "/arm",
    disarm: "/disarm",
    takeoff: "/takeoff",
    land: "/land",
    goto: "/goto",

    rtl: "/rtl",

    savePoint: "/save_point",
    savedPoints: "/saved_points"
};

let map = null;
let droneMarker = null;
let statusPollTimer = null;

document.addEventListener("DOMContentLoaded", () => {
    initMap();
    bindEvents();
    startStatusPolling();
});

function initMap() {

    map = L.map("map").setView(
        [35.6812, 139.7671],
        15
    );

    L.tileLayer(
        "https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png",
        {
            attribution:
                "&copy; OpenStreetMap contributors",
            maxZoom: 19,
        }
    ).addTo(map);

    droneMarker = L.marker(
        [35.6812, 139.7671]
    ).addTo(map);

    droneMarker.bindPopup("Drone");
}

function bindEvents() {

    document.getElementById(
        "btn-connect"
    ).addEventListener(
        "click",
        onConnect
    );

    document.getElementById(
        "btn-disconnect"
    ).addEventListener(
        "click",
        onDisconnect
    );

    document.getElementById(
        "btn-guided"
    ).addEventListener(
        "click",
        () => onSetMode("GUIDED")
    );

    document.getElementById(
        "btn-rtl"
    ).addEventListener(
        "click",
        onRTL
    );

    document.getElementById(
        "btn-land"
    ).addEventListener(
        "click",
        onLand
    );

    document.getElementById(
        "btn-arm"
    ).addEventListener(
        "click",
        onArm
    );

    document.getElementById(
        "btn-disarm"
    ).addEventListener(
        "click",
        onDisarm
    );

    document.getElementById(
        "btn-takeoff"
    ).addEventListener(
        "click",
        onTakeoff
    );

    document.getElementById(
        "btn-goto"
    ).addEventListener(
        "click",
        onGoto
    );

    const saveBtn =
        document.getElementById(
            "btn-save-point"
        );

    if (saveBtn) {

        saveBtn.addEventListener(
            "click",
            saveCurrentPoint
        );
    }
}

async function postJSON(url, data) {

    const response = await fetch(
        url,
        {
            method: "POST",
            headers: {
                "Content-Type":
                    "application/json"
            },
            body: JSON.stringify(
                data || {}
            ),
        }
    );

    const json =
        await response.json().catch(
            () => ({})
        );

    if (!response.ok) {

        throw new Error(
            json.message ||
            `Request failed: ${response.status}`
        );
    }

    return json;
}

async function getJSON(url) {

    const response =
        await fetch(url);

    return response.json();
}

function log(
    message,
    isError = false
) {

    const logArea =
        document.getElementById(
            "log-area"
        );

    const p =
        document.createElement("p");

    p.textContent =
        `[${new Date().toLocaleTimeString()}] ${message}`;

    if (isError) {
        p.style.color = "#ff8080";
    }

    logArea.appendChild(p);

    logArea.scrollTop =
        logArea.scrollHeight;
}

async function onConnect() {

    const connStr =
        document.getElementById(
            "connection-string"
        ).value.trim()
        || "tcp:127.0.0.1:5762";

    try {

        await postJSON(
            API.connect,
            {
                connection_string:
                    connStr
            }
        );

        log(
            `Connected: ${connStr}`
        );

    } catch (e) {

        log(
            e.message,
            true
        );
    }
}

async function onDisconnect() {

    try {

        await postJSON(
            API.disconnect,
            {}
        );

        log(
            "Disconnected"
        );

    } catch (e) {

        log(
            e.message,
            true
        );
    }
}

async function onSetMode(mode) {

    try {

        await postJSON(
            API.mode,
            { mode }
        );

        log(
            `Mode: ${mode}`
        );

    } catch (e) {

        log(
            e.message,
            true
        );
    }
}

async function onArm() {
    await postJSON(API.arm, {});
}

async function onDisarm() {
    await postJSON(API.disarm, {});
}

async function onRTL() {
    await postJSON(API.rtl, {});
}

async function onLand() {
    await postJSON(API.land, {});
}

async function onTakeoff() {

    const altitude =
        parseFloat(
            document.getElementById(
                "takeoff-altitude"
            ).value
        ) || 5;

    await postJSON(
        API.takeoff,
        { altitude }
    );
}

async function onGoto() {

    const latitude =
        parseFloat(
            document.getElementById(
                "goto-lat"
            ).value
        );

    const longitude =
        parseFloat(
            document.getElementById(
                "goto-lon"
            ).value
        );

    const altitude =
        parseFloat(
            document.getElementById(
                "goto-alt"
            ).value
        ) || 5;

    await postJSON(
        API.goto,
        {
            latitude,
            longitude,
            altitude
        }
    );
}

function startStatusPolling() {

    if (statusPollTimer) {
        clearInterval(
            statusPollTimer
        );
    }

    statusPollTimer =
        setInterval(
            pollStatus,
            1000
        );

    pollStatus();
}

async function pollStatus() {

    try {

        const data =
            await getJSON(
                API.status
            );

        renderStatus(data);

        await loadSavedPoints();

    } catch (e) {

    }
}

function renderStatus(data) {

    setConnectionBadge(
        !!data.connected
    );

    document.getElementById(
        "status-connected"
    ).textContent =
        data.connected
            ? "Connected"
            : "Disconnected";

    document.getElementById(
        "status-lat"
    ).textContent =
        formatValue(
            data.latitude,
            6
        );

    document.getElementById(
        "status-lon"
    ).textContent =
        formatValue(
            data.longitude,
            6
        );

    document.getElementById(
        "status-alt"
    ).textContent =
        data.relative_altitude != null
            ? `${data.relative_altitude.toFixed(1)} m`
            : "-";

    document.getElementById(
        "status-mode"
    ).textContent =
        data.flight_mode || "-";

    document.getElementById(
        "status-armed"
    ).textContent =
        data.armed
            ? "ARMED"
            : "DISARMED";

    document.getElementById(
        "status-speed"
    ).textContent =
        data.groundspeed != null
            ? `${data.groundspeed.toFixed(1)} m/s`
            : "-";

    document.getElementById(
        "status-battery"
    ).textContent =
        data.battery_voltage != null
            ? `${data.battery_voltage.toFixed(2)} V`
            : "-";

    document.getElementById(
        "status-heading"
    ).textContent =
        data.heading != null
            ? `${data.heading.toFixed(1)} °`
            : "-";

    document.getElementById(
        "status-roll"
    ).textContent =
        data.roll != null
            ? `${data.roll.toFixed(2)} °`
            : "-";

    document.getElementById(
        "status-pitch"
    ).textContent =
        data.pitch != null
            ? `${data.pitch.toFixed(2)} °`
            : "-";

    document.getElementById(
        "status-yaw"
    ).textContent =
        data.yaw != null
            ? `${data.yaw.toFixed(2)} °`
            : "-";

    if (
        data.latitude != null &&
        data.longitude != null
    ) {

        updateDroneMarker(
            data.latitude,
            data.longitude
        );
    }
}

async function saveCurrentPoint() {

    try {

        const result =
            await postJSON(
                API.savePoint,
                {}
            );

        log(
            `保存: ${result.point.latitude}, ${result.point.longitude}`
        );

        await loadSavedPoints();

    } catch (e) {

        log(
            e.message,
            true
        );
    }
}

async function loadSavedPoints() {

    const list =
        document.getElementById(
            "saved-points-list"
        );

    if (!list) {
        return;
    }

    const points =
        await getJSON(
            API.savedPoints
        );

    list.innerHTML = "";

    points.forEach(
        (point, index) => {

            const div =
                document.createElement(
                    "div"
                );

            div.className =
                "saved-point";

            div.innerHTML = `
                <strong>Point ${index + 1}</strong><br>
                Time: ${point.timestamp}<br>
                Lat: ${point.latitude}<br>
                Lon: ${point.longitude}<br>
                Alt: ${point.altitude}<br>
                Roll: ${point.roll}<br>
                Pitch: ${point.pitch}<br>
                Yaw: ${point.yaw}<br>
                Heading: ${point.heading}<br>
                Mode: ${point.flight_mode}
                <hr>
            `;

            list.appendChild(div);
        });
}

function formatValue(
    value,
    digits
) {

    return value != null
        ? Number(value).toFixed(digits)
        : "-";
}

function updateDroneMarker(
    lat,
    lon
) {

    const latLng = [
        lat,
        lon
    ];

    droneMarker.setLatLng(
        latLng
    );

    map.panTo(
        latLng,
        {
            animate: true
        }
    );
}

function setConnectionBadge(
    connected
) {

    const badge =
        document.getElementById(
            "connection-status"
        );

    if (connected) {

        badge.textContent =
            "Connected";

        badge.className =
            "status-badge connected";

    } else {

        badge.textContent =
            "Disconnected";

        badge.className =
            "status-badge disconnected";
    }
}