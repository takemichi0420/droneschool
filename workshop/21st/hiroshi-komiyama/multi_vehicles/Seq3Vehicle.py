#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Seq3Vehicle.py

複数機体順次制御

Rover → Boat → Copter

課題仕様:
1. Rover
   ARM
   AUTO
   WP6到達待ち

2. Boat
   ARM
   AUTO
   WP9到達待ち

3. Copter
   ARM
   GUIDED
   Takeoff 10m
   AUTO
   WP10到達
   LAND
   DISARM


前提:
- SITLは別途起動済み
- Mission Planner接続済み
- ミッションは各機体へ登録済み
"""


from pymavlink import mavutil

import time
import math


# ==========================================================
# 接続設定
# ==========================================================

HOST = "192.168.1.13"

#ROVER_PORT = 5760
#BOAT_PORT = 5770
#COPTER_PORT = 5780
ROVER_PORT = 5762
BOAT_PORT = 5772
COPTER_PORT = 5782


SOURCE_SYSTEM = 1
SOURCE_COMPONENT = 90


# ==========================================================
# 制御設定
# ==========================================================

#WAYPOINT_THRESHOLD = 5.0     # 到達距離 m
WAYPOINT_THRESHOLD = 10.0     # 到達距離 m
CONFIRM_COUNT = 5             # 連続確認回数

TAKEOFF_ALTITUDE = 10.0       # Copter離陸高度


EARTH_RADIUS = 6371000


# ==========================================================
# WayPoint 座標
# ==========================================================


WAYPOINTS = {

    # Rover
    "rover_home":
        (35.876991, 140.348026),

    "rover_wp6":
        (35.879723549814685,
         140.34844993792788),


    # Boat
    "boat_home":
        (35.879768, 140.348495),

    "boat_wp9":
        (35.8782589277083,
         140.33804498166177),


    # Copter
    "copter_home":
        (35.878275,
         140.338069),

    "goal":
        (35.877518,
         140.295439)

}
"""
    "goal":
        (35.877518,
         140.295439)
        (35.878131583036534,
         140.33532128807087),
"""


# ==========================================================
# 距離計算
# ==========================================================


def calculate_distance(
        lat1,
        lon1,
        lat2,
        lon2):

    """
    Haversine距離計算
    """

    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)

    dlat = lat2 - lat1
    dlon = math.radians(lon2 - lon1)


    a = (
        math.sin(dlat / 2) ** 2
        +
        math.cos(lat1)
        *
        math.cos(lat2)
        *
        math.sin(dlon / 2) ** 2
    )


    c = 2 * math.atan2(
        math.sqrt(a),
        math.sqrt(1-a)
    )


    return EARTH_RADIUS * c



# ==========================================================
# Vehicle Controller
# ==========================================================


class VehicleController:


    def __init__(
            self,
            name,
            connection,
            vehicle_type):

        self.name = name
        self.connection = connection
        self.vehicle_type = vehicle_type

        self.master = None



    # ------------------------------------------------------
    # Connect
    # ------------------------------------------------------

    def connect(self):

        print(
            f"[{self.name}] Connecting..."
        )


        self.master = mavutil.mavlink_connection(
            self.connection,
            source_system=SOURCE_SYSTEM,
            source_component=SOURCE_COMPONENT
        )


        self.master.wait_heartbeat()


        print(
            f"[{self.name}] Heartbeat OK"
        )

        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10,
            1
        )


    # ------------------------------------------------------
    # Mode変更
    # ------------------------------------------------------

    def change_mode(self, mode):

        print(
            f"[{self.name}] MODE -> {mode}"
        )


        mode_id = self.master.mode_mapping()[mode]


        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )


        while True:

            msg = self.master.recv_match(
                type="HEARTBEAT",
                blocking=True
            )

            if msg:

                current = (
                    self.master.flightmode
                )

                if current == mode:
                    break


        print(
            f"[{self.name}] Mode changed"
        )



    # ------------------------------------------------------
    # ARM
    # ------------------------------------------------------

    def arm(self):

        print(
            f"[{self.name}] ARM"
        )


        self.master.mav.command_long_send(

            self.master.target_system,
            self.master.target_component,

            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,

            0,

            1,

            0,0,0,0,0,0

        )


        self.master.motors_armed_wait()


        print(
            f"[{self.name}] Armed"
        )
# ==========================================================
# Takeoff (Copter)
# ==========================================================
    """
    def takeoff(self, altitude):

        print(
            f"[{self.name}] TAKEOFF {altitude}m"
        )


        self.master.mav.command_long_send(

            self.master.target_system,
            self.master.target_component,

            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,

            0,

            0,
            0,
            0,
            0,
            0,
            0,

            altitude

        )

        print("Loop前")

        while True:

            msg = self.master.recv_match(
#                type="GLOBAL_POSITION_INT",
                blocking=True,
                timeout=2
            )

            if msg is None:
                print("Timeout")
                continue
            
            if msg:

                current_alt = (
                    msg.relative_alt / 1000.0
                )


                print(
                    f"[{self.name}] ALT {current_alt:.1f}m"
                )


                if current_alt >= altitude * 0.95:
                    break


            time.sleep(0.2)



        print(
            f"[{self.name}] Takeoff complete"
        )
    """

    def takeoff(self, altitude):

        print(
            f"[{self.name}] TAKEOFF {altitude}m"
        )


        self.master.mav.command_long_send(

            self.master.target_system,
            self.master.target_component,

            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,

            0,

            0,
            0,
            0,
            0,
            0,
            0,

            altitude

        )


        while True:

#            msg = self.master.recv_match(
#                blocking=True,
#                timeout=2
#            )

            msg = self.master.recv_match(
               type="GLOBAL_POSITION_INT",
                blocking=True
            )            
   
            if msg is None:
                print("Timeout")
                continue
        
#            print(msg.get_type())
#            if msg.get_type() != "GLOBAL_POSITION_INT":
#                continue

            current_alt = msg.relative_alt / 1000.0

            print(f"ALT {current_alt:.1f}")

            if current_alt >= altitude * 0.95:
                break


            time.sleep(0.2)



        print(
            f"[{self.name}] Takeoff complete"
        )


# ==========================================================
# Mission開始
# ==========================================================


    def start_mission(self):

        self.change_mode("AUTO")



# ==========================================================
# WayPoint到達待ち
# ==========================================================


    def wait_waypoint(
            self,
            target_lat,
            target_lon,
            wp_name):


        print(
            f"[{self.name}] Waiting {wp_name}"
        )


        count = 0
        last_distance = None


        while True:

            msg = self.master.recv_match(
                type="GLOBAL_POSITION_INT",
                blocking=True
            )


            if not msg:
                continue



            lat = msg.lat / 1e7
            lon = msg.lon / 1e7


            distance = calculate_distance(
                lat,
                lon,
                target_lat,
                target_lon
            )



            if (
                last_distance is None
                or abs(distance-last_distance) > 5
            ):

                print(
                    f"[{self.name}] "
                    f"{wp_name}: {distance:.1f}m"
                )

                last_distance = distance



            if distance < WAYPOINT_THRESHOLD:

                count += 1

                if count >= CONFIRM_COUNT:

                    print(
                        f"[{self.name}] "
                        f"{wp_name} reached"
                    )

                    return


            else:

                count = 0



            time.sleep(0.1)




# ==========================================================
# LAND
# ==========================================================


    def land(self):

        print(
            f"[{self.name}] LAND"
        )


        self.change_mode("LAND")



        while True:

            msg = self.master.recv_match(
                type="GLOBAL_POSITION_INT",
                blocking=True
            )


            if msg:

                alt = (
                    msg.relative_alt / 1000
                )


                print(
                    f"[{self.name}] ALT {alt:.1f}m"
                )


                if alt < 0.3:

                    break



        print(
            f"[{self.name}] Landed"
        )



# ==========================================================
# DISARM
# ==========================================================


    def disarm(self):

        print(
            f"[{self.name}] DISARM"
        )


        self.master.mav.command_long_send(

            self.master.target_system,
            self.master.target_component,

            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,

            0,

            0,

            0,0,0,0,0,0

        )


        print(
            f"[{self.name}] Disarmed"
        )



# ==========================================================
# Close
# ==========================================================


    def close(self):

        if self.master:

            self.master.close()



# ==========================================================
# Main
# ==========================================================

def main():


    print("=" * 50)
    print(
        " Multi Vehicle Sequential Controller "
    )
    print("=" * 50)



    rover = VehicleController(
        "ROVER",
        f"tcp:{HOST}:{ROVER_PORT}",
        "rover"
    )


    boat = VehicleController(
        "BOAT",
        f"tcp:{HOST}:{BOAT_PORT}",
        "boat"
    )


    copter = VehicleController(
        "COPTER",
        f"tcp:{HOST}:{COPTER_PORT}",
        "copter"
    )



    try:


        # --------------------------------------
        # 全機体接続
        # --------------------------------------

        rover.connect()

        boat.connect()

        copter.connect()


        print(
            "\nAll Vehicles Connected.\n"
        )



        # --------------------------------------
        # Rover
        # --------------------------------------

        print(
            "\n========== Rover START =========="
        )


        rover.arm()

        rover.start_mission()


        rover.wait_waypoint(

            *WAYPOINTS["rover_wp6"],

            "WP6"

        )



        print(
            "Rover Complete"
        )



        # --------------------------------------
        # Boat
        # --------------------------------------

        print(
            "\n========== Boat START =========="
        )


        boat.arm()

        boat.start_mission()


        boat.wait_waypoint(

            *WAYPOINTS["boat_wp9"],

            "WP9"

        )


        print(
            "Boat Complete"
        )



        # --------------------------------------
        # Copter
        # --------------------------------------

        print(
            "\n========== Copter START =========="
        )


        copter.arm()


        copter.change_mode(
            "GUIDED"
        )


        copter.takeoff(
            TAKEOFF_ALTITUDE
        )

        time.sleep(10)      #離陸10mまで少し待つ

        copter.start_mission()


        copter.wait_waypoint(

            *WAYPOINTS["goal"],

            "GOAL"

        )


        copter.land()


        copter.disarm()



        print(
            "\nALL VEHICLES COMPLETE"
        )



    except Exception as e:

        print(
            "\nERROR:"
        )

        print(e)



    finally:

        rover.close()
        boat.close()
        copter.close()

"""
def main():


    print("=" * 50)
    print(
        " Multi Vehicle Sequential Controller "
    )
    print("=" * 50)



    copter = VehicleController(
        "COPTER",
        f"tcp:{HOST}:{COPTER_PORT}",
        "copter"
    )



    try:


        # --------------------------------------
        # 全機体接続
        # --------------------------------------

   
        copter.connect()


        print(
            "\nAll Vehicles Connected.\n"
        )


        # --------------------------------------
        # Copter
        # --------------------------------------

        print(
            "\n========== Copter START =========="
        )


        copter.arm()


        copter.change_mode(
            "GUIDED"
        )


        copter.takeoff(
            TAKEOFF_ALTITUDE
        )


        copter.start_mission()


        copter.wait_waypoint(

            *WAYPOINTS["goal"],

            "GOAL"

        )

        print("Waypoint reached")
        print("Start LAND")

        copter.land()

        print("LAND command sent")


        copter.disarm()



        print(
            "\nALL VEHICLES COMPLETE"
        )



    except Exception as e:

        print(
            "\nERROR:"
        )

        print(e)



    finally:

        copter.close()
"""

if __name__ == "__main__":

    main()