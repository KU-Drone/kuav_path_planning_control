from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command, LocationLocal, Vehicle, Locations
import time
import math
from pymavlink import mavutil
from pymavlink.dialects.v20.common import MAVLink
import pymap3d as pm
from coordinate_changer import egm96ToEllipsoid, ellipsoidToEgm96
ATTITUDE_QUATERNION = 31
EXTENDED_SYS_STATE = 245
GPS_GLOBAL_ORIGIN = 49
GLOBAL_POSITION_INT = 33


ekf_origin=None

def request_ekf3_origin(vehicle: Vehicle):
    vehicle.wait_for_armable()

    # print("requesting ekf origin")
    vehicle.message_factory.command_long_send(0,0, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0, GPS_GLOBAL_ORIGIN, 0, 0, 0, 0, 0, 0)
    ekf_origin = vehicle._master.recv_match(type='GPS_GLOBAL_ORIGIN', blocking=True)
    return (ekf_origin.latitude*1e-7, ekf_origin.longitude*1e-7, ekf_origin.altitude*1e-3)

def request_global_position(vehicle: Vehicle):
    vehicle.wait_for_armable()

    vehicle.message_factory.command_long_send(0,0, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0, GLOBAL_POSITION_INT, 0, 0, 0, 0, 0, 0)
    global_position = vehicle._master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    # print(global_position)
    return (global_position.lat*1e-7, global_position.lon*1e-7, global_position.alt*1e-3)


def ned_to_global(n, e, d, ekf_origin):
    lat, lon, alt = ekf_origin
    return pm.ned2geodetic(n, e, d, lat, lon, alt)

def global_to_ned(lat, lon, alt, ekf_origin):
    lat0, lon0, h0 = ekf_origin
    return pm.geodetic2ned(lat, lon, alt, lat0, lon0, h0)

def global_position_callback(self, name, message):
    print(message)
    global_position_msl = (message.lat*1e-7, message.lon*1e-7, message.alt*1e-3)
    global_position_ellipsoid = egm96ToEllipsoid(*global_position_msl)
    # print(global_position)
    print(global_to_ned(*global_position_msl, ekf_origin_msl))
    print(global_to_ned(*global_position_ellipsoid, ekf_origin_ellipsoid))

#    MAVLink(file).command_long_send(target_system, target_component, command, confirmation, param1, param2, param3, param4, param5, param6, param7)
if __name__ == "__main__":
    vehicle = connect("udpin:127.0.0.1:14550")

    # #{'velocity', 'location.global_relative_frame', 'location.global_frame', 'battery', 'airspeed', 'groundspeed', 'last_heartbeat', 'attitude', 'autopilot_version', 'heading', 'location', 'gps_0', 'location.local_frame', 'home_location'}

    # #position callback
    # vehicle.wait_for_armable()
    # ekf_origin_msl = request_ekf3_origin(vehicle)
    # ekf_origin_ellipsoid = egm96ToEllipsoid(*ekf_origin_msl)
    # ekf_origin = ekf_origin_ellipsoid
    # vehicle.add_message_listener("GLOBAL_POSITION_INT", global_position_callback)
    # vehicle.message_factory.command_long_send(0,0, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, GLOBAL_POSITION_INT, 250000, 0, 0, 0, 0, 0)
    # while True:
    #     pass
    
    # #takeoff
    # vehicle.wait_for_armable()
    # vehicle.mode = VehicleMode("TAKEOFF") #takes off depending on pitch (low-fixed wing, high-vtol)
    # vehicle.arm()
    # vehicle.wait_for_alt(1)
    # print("takeoff done")
    # # vehicle.wait_simple_takeoff(alt=40) #takes off vtol mode
    # # vehicle.message_factory.command_long_send(0,0, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, float("nan"), 0, 0, 40) #takes off vtol mode

    

    # MAVLink().mission_item_int_send(target_system, target_component, seq, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z)
    ekf_origin_msl = request_ekf3_origin(vehicle)
    ekf_origin_ellipsoid = egm96ToEllipsoid(*ekf_origin_msl)
    vehicle.mode = VehicleMode("GUIDED")
    lat, lon, alt = ellipsoidToEgm96(*ned_to_global(100, -100, -40, ekf_origin_ellipsoid))
    message = vehicle.message_factory.mission_item_int_send( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 0, 0, 0, 0, int(lat*1e+7), int(lon*1e+7), int(alt))

    # # message = vehicle.message_factory.mission_item_int_encode(0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 0, 0, 0, 0, 0, 0, -20)

    # vehicle.send_mavlink(message)
    # connection.wait_heartbeat()
    # # connection.mav.message_interval_send(ATTITUDE_QUATERNION, 250000)
    # connection.mav.command_long_send(connection.target_system, connection.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, ATTITUDE_QUATERNION, 250000, 0, 0, 0, 0, 0)
    # last_time = time.perf_counter()

    # while True:
    #     try:
    #         # print(connection.messages.keys())
    #         print(connection.recv_match(type='ATTITUDE_QUATERNION', blocking=True))
    #         print(connection.time_since('ATTITUDE_QUATERNION'))
    #         t = time.perf_counter()
    #         print(1/(t-last_time))
    #         last_time = t
    #     except KeyError:
    #         pass


    # vehicle.message_factory.command_long_send(0,0, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, GPS_GLOBAL_ORIGIN, 250000, 0, 0, 0, 0, 0)
    # last_time = time.perf_counter()
    # while True:
    #     try:
    #         # print(connection.messages.keys())
    #         print(vehicle._master.recv_match(type='GPS_GLOBAL_ORIGIN', blocking=True))
    #         print(vehicle._master.time_since('GPS_GLOBAL_ORIGIN'))
    #         t = time.perf_counter()
    #         print(1/(t-last_time))
    #         last_time = t
    #     except KeyError:
    #         pass

