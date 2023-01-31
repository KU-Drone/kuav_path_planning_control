from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command, LocationLocal
import time
import math
from pymavlink import mavutil
from pymavlink.dialects.v20.common import MAVLink

vehicle = connect("udpin:127.0.0.1:14550")

vehicle.wait_for_mode("GUIDED")

vehicle.wait_for_armable()

vehicle.arm()

vehicle.wait_simple_takeoff(alt=40)

message = vehicle.message_factory.mission_item_int_encode(0, 0, 0, 0, 16, 2, 0, 0, 0, 0, 0, -353621474, 1491651746, 700)

# message = vehicle.message_factory.mission_item_int_encode(0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 0, 0, 0, 0, 0, 0, -20)

vehicle.send_mavlink(message)

