# Multi Vehicle Sequential Control

## 概要

本プログラムは、ArduPilot SITL 上で **Rover・Boat・Copter** の3機体を順番に制御する Python プログラムです。

物流センターを想定したシナリオとして、

* Rover が荷物を Boat 乗り場まで運搬
* Boat が川を渡って荷物を運搬
* Copter が最終目的地まで配送

を自動で実行します。

---

# 動作環境

* ArduPilot SITL
* Mission Planner
* Python 3
* pymavlink

---

# 使用ライブラリ

* pymavlink
* math
* time

---

# 機体構成

| Vehicle | SYSID | Mission Planner |   Python |
| ------- | ----: | --------------: | -------: |
| Rover   |     1 |        TCP 5760 | TCP 5762 |
| Boat    |     2 |        TCP 5770 | TCP 5772 |
| Copter  |     3 |        TCP 5780 | TCP 5782 |

Mission Planner は各機体へ以下の TCP ポートで接続します。

* Rover：5760
* Boat：5770
* Copter：5780

Python プログラムは内部制御用 TCP ポートへ接続します。

* Rover：5762
* Boat：5772
* Copter：5782

各機体では Mission Planner 用ポートとは別に、内部制御用として「+2」の TCP ポートが使用されるため、本プログラムでは 5762、5772、5782 を使用して通信します。

---

# ファイル構成

```text
Seq3Vehicle.py
README.md
MissionWayPoint_01Rover.txt
MissionWayPoint_02Boat.txt
MissionWayPoint_03Copter.txt
```

---

# Waypointファイル

### MissionWayPoint_01Rover.txt

Rover 用ミッションです。

* Home
* WP1
* WP2
* WP3
* WP4
* WP5
* WP6

Rover が WP6 に到達すると、Python プログラムが Boat のミッションを開始します。

---

### MissionWayPoint_02Boat.txt

Boat 用ミッションです。

* Home
* WP7
* WP8
* WP9

Boat が WP9 に到達すると、Python プログラムが Copter のミッションを開始します。

---

### MissionWayPoint_03Copter.txt

Copter 用ミッションです。

* Home
* Goal

Copter は Python プログラムにより **GUIDED モードで離陸**した後、**AUTO モードへ切り替わり、高度10mを維持したまま Goal まで自動飛行**します。

Goal 到達後は LAND モードへ切り替わり、自動着陸します。

---

# 実行手順

## 1. SITL を起動

Rover、Boat、Copter の3機体を起動します。

---

## 2. Mission Planner

各機体へ接続し、

* MissionWayPoint_01Rover.txt
* MissionWayPoint_02Boat.txt
* MissionWayPoint_03Copter.txt

をそれぞれの機体へ書き込みます。

---

## 3. Python プログラムを実行

```bash
python3 Seq3Vehicle.py
```

---

# 動作シーケンス

1. Rover、Boat、Copter に接続
2. Rover を ARM
3. Rover を AUTO モードへ変更
4. Rover が WP6 に到達
5. Boat を ARM
6. Boat を AUTO モードへ変更
7. Boat が WP9 に到達
8. Copter を ARM
9. Copter を GUIDED モードへ変更
10. 高度10mまで離陸
11. AUTO モードへ変更
12. 高度10mを維持して Goal まで飛行
13. Goal 到達
14. LAND モードへ変更
15. 自動着陸
16. Disarm
17. プログラム終了

---

# Waypoint 到達判定

Waypoint の到達判定には GPS 座標を使用します。

* 到達半径：10m
* 連続確認回数：5回

機体が Waypoint から10m以内に入り、その状態が5回連続で確認された場合に到達と判定し、次の機体の制御へ移行します。

---

# 特徴

* Rover → Boat → Copter を順番に自動制御
* GPS 座標による Waypoint 到達判定
* TCP による MAVLink 通信
* Copter は GUIDED モードで離陸後、AUTO モードへ切り替え
* 高度10mを維持したまま Goal へ飛行
* Goal 到達後は LAND モードへ切り替え、自動着陸
* Python のみで3機体を連携制御

---

# 注意事項

* 3機体の SITL を事前に起動してください。
* Mission Planner で各機体へ対応する Waypoint ファイルを書き込んでから実行してください。
* Python 内で使用する Waypoint 座標と Mission Planner の Waypoint 座標は一致させてください。
* Copter の Goal の高度は **10m** に設定してください。

---

# 実行例

```text
========== Rover START ==========
[ROVER] ARM
[ROVER] MODE -> AUTO
[ROVER] WP6 reached

========== Boat START ==========
[BOAT] ARM
[BOAT] MODE -> AUTO
[BOAT] WP9 reached

========== Copter START ==========
[COPTER] ARM
[COPTER] MODE -> GUIDED
[COPTER] TAKEOFF 10.0m
[COPTER] Takeoff complete
[COPTER] MODE -> AUTO
[COPTER] COPTER GOAL reached
[COPTER] LAND
[COPTER] Landed

ALL VEHICLES COMPLETE
```
