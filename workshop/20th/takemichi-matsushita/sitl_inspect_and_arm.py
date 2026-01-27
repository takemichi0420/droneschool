#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
SITL用: 自動点検 -> ARM可否判定 -> (PASSなら) ARM要求まで実施

- 自動点検: SYS_STATUS, GPS_RAW_INT, EKF_STATUS_REPORT, STATUSTEXT(PreArm)
- ARM可否: 実際に MAV_CMD_COMPONENT_ARM_DISARM を投げて COMMAND_ACK を確認
  (拒否されたらSTATUSTEXTから理由を拾う)
"""

import argparse
import time
from typing import Optional, List

from pymavlink import mavutil


DEFAULTS = {
    "battery_min_v": 10.0,     # SITLは電圧モデルが怪しい場合があるので低めデフォ
    "gps_min_fix": 3,
    "gps_min_sats": 6,
    "ekf_required": True,
    "prearm_wait_s": 30,       # PreArmが消えるのを待つ最大時間
}


def recv_latest(master, msg_type: str, timeout_s: float = 1.0):
    t0 = time.time()
    last = None
    while time.time() - t0 < timeout_s:
        m = master.recv_match(type=msg_type, blocking=False)
        if m is not None:
            last = m
        time.sleep(0.02)
    return last


def collect_statustext(master, duration_s: float = 1.0) -> List[str]:
    out = []
    t0 = time.time()
    while time.time() - t0 < duration_s:
        m = master.recv_match(type="STATUSTEXT", blocking=False)
        if m is not None:
            txt = m.text.decode(errors="ignore") if isinstance(m.text, (bytes, bytearray)) else str(m.text)
            out.append(txt)
        time.sleep(0.02)
    return out


def has_prearm_error(texts: List[str]) -> bool:
    # "PreArm:" が含まれる警告/エラーをざっくり検出
    # （SITL/実機で文言は変わるので、運用では正規表現を強化してOK）
    for t in texts:
        if "PreArm" in t or "Prearm" in t or "prearm" in t:
            # OK系メッセージもあるが、基本はPreArmが出ている時点でARM阻害の可能性が高い
            # ここでは「PreArmが残っている間は待つ」戦略を取る
            return True
    return False


def wait_prearm_clear(master, max_wait_s: float) -> List[str]:
    """PreArmメッセージが落ち着くのを待つ。最後に観測したSTATUSTEXTを返す。"""
    last_texts: List[str] = []
    t0 = time.time()
    while time.time() - t0 < max_wait_s:
        texts = collect_statustext(master, duration_s=1.0)
        if texts:
            last_texts = texts
        if not has_prearm_error(texts):
            return last_texts
        time.sleep(0.5)
    return last_texts


def request_message_rates(master):
    # SITLならなくても来ることが多いですが、安定化のため要求
    # MAV_DATA_STREAM_ALL: 0
    try:
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            4,  # Hz
            1
        )
    except Exception:
        pass


def get_arm_ack(master, timeout_s: float = 5.0) -> Optional[int]:
    """ARM/DISARMのCOMMAND_ACK(result)を待つ。resultは MAV_RESULT_*"""
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        m = master.recv_match(type="COMMAND_ACK", blocking=False)
        if m is not None and int(m.command) == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            return int(m.result)
        time.sleep(0.02)
    return None


def get_param_int(master, name: str, timeout_s: float = 3.0) -> Optional[int]:
    """PARAM_VALUEから指定パラメータを取得してintで返す。"""
    try:
        master.mav.param_request_read_send(
            master.target_system,
            master.target_component,
            name.encode("ascii"),
            -1,
        )
    except Exception:
        return None

    t0 = time.time()
    while time.time() - t0 < timeout_s:
        m = master.recv_match(type="PARAM_VALUE", blocking=False)
        if m is None:
            time.sleep(0.02)
            continue
        pid = m.param_id.decode(errors="ignore") if isinstance(m.param_id, (bytes, bytearray)) else str(m.param_id)
        if pid.strip("\x00") == name:
            try:
                return int(m.param_value)
            except Exception:
                return None
        time.sleep(0.02)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--conn", default="udp:127.0.0.1:14550")
    ap.add_argument("--arm", action="store_true", help="PASSならARM要求まで行う")
    ap.add_argument("--battery-min-v", type=float, default=DEFAULTS["battery_min_v"])
    ap.add_argument("--gps-min-fix", type=int, default=DEFAULTS["gps_min_fix"])
    ap.add_argument("--gps-min-sats", type=int, default=DEFAULTS["gps_min_sats"])
    ap.add_argument("--no-ekf-required", action="store_true", help="EKF必須を外す（SITLが安定しない場合用）")
    ap.add_argument("--prearm-wait-s", type=float, default=DEFAULTS["prearm_wait_s"])
    args = ap.parse_args()

    ekf_required = not args.no_ekf_required

    master = mavutil.mavlink_connection(args.conn, autoreconnect=True, timeout=5)
    hb = master.wait_heartbeat(timeout=15)
    if hb is None:
        raise SystemExit("HEARTBEATが来ません。SITL起動/接続先を確認してください。")

    print(f"[OK] ハートビート受信: sys={master.target_system} comp={master.target_component}")
    request_message_rates(master)
    ekf_action_val = get_param_int(master, "FS_EKF_ACTION", timeout_s=3.0)
    ekf_action_map = {
        1: "その場で着陸",
        2: "ホバリング",
        3: "ホームポイントに戻る",
    }
    ekf_action_label = ekf_action_map.get(ekf_action_val, f"不明({ekf_action_val})") if ekf_action_val is not None else "不明"

    # PreArmが落ち着くのを待つ（SITLでも起動直後はEKF/GPS待ちが出る）
    last_prearm = wait_prearm_clear(master, max_wait_s=args.prearm_wait_s)
    if last_prearm:
        # 参考として最後のメッセージを表示
        print("[INFO] 最新のSTATUSTEXT（直近）:")
        for t in last_prearm[-5:]:
            print("  -", t)

    # 主要テレメトリ取得
    sys_status = recv_latest(master, "SYS_STATUS", timeout_s=2.0)
    battery_status = recv_latest(master, "BATTERY_STATUS", timeout_s=2.0)
    gps = recv_latest(master, "GPS_RAW_INT", timeout_s=2.0)
    ekf = recv_latest(master, "EKF_STATUS_REPORT", timeout_s=2.0)

    # 判定
    reasons = []
    ok = True

    # Battery
    batt_v = sys_status.voltage_battery / 1000.0 if sys_status and sys_status.voltage_battery is not None else None
    batt_rem = None
    if sys_status and sys_status.battery_remaining is not None:
        br = int(sys_status.battery_remaining)
        batt_rem = None if br < 0 else br
    if batt_v is not None:
        if batt_v < args.battery_min_v:
            ok = False
            reasons.append(f"バッテリー電圧が低い: {batt_v:.2f}V < {args.battery_min_v:.2f}V")
    else:
        reasons.append("バッテリー電圧が不明（SYS_STATUS未受信）")
    cell_voltages_v = []
    cell_delta_v = None
    cell_balance_label = "不明"
    if battery_status and getattr(battery_status, "voltages", None):
        raw_cells = list(battery_status.voltages)
        for mv in raw_cells:
            # 65535や0は未使用セルの値なので除外
            if mv is None or mv <= 0 or mv >= 65535:
                continue
            cell_voltages_v.append(mv / 1000.0)
        if len(cell_voltages_v) >= 2:
            cell_delta_v = max(cell_voltages_v) - min(cell_voltages_v)
            if cell_delta_v <= 0.03:
                cell_balance_label = "正常"
            elif cell_delta_v <= 0.05:
                cell_balance_label = "注意"
            else:
                cell_balance_label = "異常"

    # GPS
    FIX_STR = {0: "NO_GPS", 1: "NO_FIX", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK_FLOAT", 6: "RTK_FIXED"}
    gps_fix = int(gps.fix_type) if gps else None
    gps_fix_str = FIX_STR.get(gps_fix, f"UNKNOWN({gps_fix})") if gps_fix is not None else "N/A"
    gps_sats = int(gps.satellites_visible) if gps and gps.satellites_visible is not None else None
    gps_hdop = (gps.eph / 100.0) if gps and gps.eph is not None else None
    if gps_fix is not None:
        if gps_fix < args.gps_min_fix:
            ok = False
            reasons.append(f"GPSのfixが不足: fix={gps_fix} < {args.gps_min_fix}")
        if gps_sats is not None and gps_sats < args.gps_min_sats:
            ok = False
            reasons.append(f"GPSの衛星数が不足: sats={gps_sats} < {args.gps_min_sats}")
    else:
        ok = False
        reasons.append("GPSが不明（GPS_RAW_INT未受信）")

    # EKF
    ekf_ok = None
    if ekf:
        # 簡易：flags!=0をOK扱い（運用ではビット定義に合わせて強化）
        ekf_ok = int(ekf.flags) != 0
        if ekf_required and not ekf_ok:
            ok = False
            reasons.append(f"EKFが不健康（flags={int(ekf.flags)}）")
    else:
        if ekf_required:
            ok = False
            reasons.append("EKFが不明（EKF_STATUS_REPORT未受信）")

    # 直近STATUSTEXTも拾っておく（ARM拒否理由が出ることが多い）
    statustext = collect_statustext(master, duration_s=1.5)

    # 結果表示
    print("\n=== 自動点検結果 ===")
    print(
        f"バッテリー: {batt_v if batt_v is not None else 'N/A'} V, "
        f"残量={batt_rem if batt_rem is not None else 'N/A'} %（下限 {args.battery_min_v}V）"
    )
    if cell_voltages_v:
        cells_str = ", ".join(f"{v:.2f}V" for v in cell_voltages_v)
        delta_str = f"{cell_delta_v:.2f}V" if cell_delta_v is not None else "N/A"
        print(f"  セル電圧: [{cells_str}] / セル差={delta_str} / 判定={cell_balance_label}")
    else:
        print("  セル電圧: 取得不可（BATTERY_STATUS未受信またはセル情報なし）")
    print(
        f"GNSS: fix={gps_fix_str}, sats={gps_sats if gps_sats is not None else 'N/A'}, "
        f"hdop~={gps_hdop if gps_hdop is not None else 'N/A'} "
        f"（最小 fix {args.gps_min_fix}, sats {args.gps_min_sats}）"
    )
    print("  ※ fix=測位モード（例: 3D/RTK_FIXED）、sats=衛星数、hdop=水平位置精度の目安（小さいほど良い）")
    print(
        f"自動制御系統（EKF）: flags={int(ekf.flags) if ekf else 'N/A'}（必須={ekf_required}）, "
        f"フェイルセーフ動作={ekf_action_label}"
    )
    print("  ※ flags=EKFの合格項目ビット列（831=姿勢/速度/位置/高度/予測位置がOK）")
    print("STATUSTEXT（直近）:")
    for t in statustext[-5:]:
        print("  -", t)

    if not ok:
        print("\n[FAIL] 点検に失敗しました。理由:")
        for r in reasons:
            print(" -", r)
        print("\nARM判定: ARM不可（スクリプト判定）")
        raise SystemExit(2)

    print("\n[PASS] 点検に合格しました。")
    print("ARM（モーター始動）判定: ARM可能（スクリプト判定）")

    if not args.arm:
        return

    # PASSならARMを試行（可否を確定）
    print("\n[TRY] ARMコマンドを送信します...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,  # param1=1 arm
        0, 0, 0, 0, 0, 0
    )

    ack = get_arm_ack(master, timeout_s=5.0)
    if ack is None:
        print("[WARN] ARMのCOMMAND_ACKが未受信です。HEARTBEATのARM状態を確認します...")
        time.sleep(1.0)

    # armed state確認
    hb2 = recv_latest(master, "HEARTBEAT", timeout_s=2.0)
    armed = master.motors_armed()  # pymavlink helper
    if armed:
        print("[OK] 機体はARM済みです。")
        raise SystemExit(0)

    # 失敗時はSTATUSTEXTを追加収集して理由を出す
    more = collect_statustext(master, duration_s=2.0)
    print("[NG] 機体はARMできませんでした。")
    if ack is not None:
        print(f"COMMAND_ACK結果: {ack}（MAV_RESULT）")
    if more:
        print("STATUSTEXT（ARM試行後）:")
        for t in more[-10:]:
            print("  -", t)
    raise SystemExit(3)


if __name__ == "__main__":
    main()
