#!/usr/bin/env python
# encoding: utf-8

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Attitude
from pymavlink import mavutil # Needed for command message definitions
import time
import math

# Connect to the Vehicle
vehicle = connect("127.0.0.1:14550", wait_ready=True)
#vehicle = connect("com9,115200", wait_ready=True)
#vehicle = connect("/dev/ttyUSB0,921600", wait_ready=True)

att_history = []

def location_relative_callback(self, name, val):
    print(" relative location changed ")
    print(name)
    print(val)

def location_global_callback(self, name, val):
    print(" global location changed ")
    print(name)
    print(val)

def attitude_callback(self, name, val):
    print(" attitude changed ")
    print(name)
    print(val)
    ts = time.time


# vehicle.add_attribute_listener('location.global_relative_frame', location_relative_callback)
# vehicle.add_attribute_listener('location.global_frame', location_global_callback)
# vehicle.add_attribute_listener('attitude', attitude_callback)


def arm_vechile():
    # Armed状態の場合は処理しない
    if vehicle.armed:
        return
    
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)


# Armする
# arm_vechile()


# Yawを変更する関数
def condition_yaw(heading, relative=False):
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

# 距離から新しい座標を返す
def get_location_metres(original_location, dNorth, dEast):
    earth_radius = 6378137.0 # Radius of "spherical" earth
    # Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    # New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
    
    return targetlocation

# 座標間距離を返す
def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

# 方位近似計算
def get_bearing(aLocation1, aLocation2):
    off_x = aLocation2.lon - aLocation1.lon
    off_y = aLocation2.lat - aLocation1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing

# 座標移動
def goto_position_target_global_int(aLocation):
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111111000, # type_mask (only speeds enabled)
        aLocation.lat*1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
        aLocation.lon*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        aLocation.alt, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0, # X velocity in NED frame in m/s
        0, # Y velocity in NED frame in m/s
        0, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)

# NED移動
def goto_position_target_local_ned(north, east, down):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)

# 北、東移動
def goto(dNorth, dEast, gotoFunction=vehicle.simple_goto):    
    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)

    while vehicle.mode.name == "GUIDED":
        remainingDistance = get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        print("Distance to target: ", remainingDistance)
        if remainingDistance <= targetDistance * 0.01: #Just below target, in case of undershoot.
            print("Reached target")
            break
        time.sleep(2)

# NED速度移動
def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle
    for _ in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)
    
    


def send_global_velocity(velocity_x, velocity_y, velocity_z, duration):
    # ファームのバージョンによって動作が異なる
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x, # X velocity in NED frame in m/s
        velocity_y, # Y velocity in NED frame in m/s
        velocity_z, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle
    for _ in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)    


# # Vehicle.simple_goto()関数を使った移動

# print("Set groundspeed to 5m/s.")
# vehicle.groundspeed=5

# print("Position North 80 West 50")
# goto(80, -50)

# print("Position North 0 East 100")
# goto(0, 100)

# print("Position North -80 West 50")
# goto(-80, -50)


# # 座標を使った移動
# print("Set groundspeed to 5m/s.")
# vehicle.groundspeed = 5
# goto(-100, -130, goto_position_target_global_int)

# print("Set groundspeed to 15m/s (max).")
# vehicle.groundspeed = 15
# print("Position South 0 East 200")
# goto(0, 260, goto_position_target_global_int)

# print("Position North 100 West 130")
# goto(100, -130, goto_position_target_global_int)


# Yaw移動
DURATION = 20 #Set duration for each segment.
# Set up velocity vector to map to each direction.
# vx > 0 => fly North
# vx < 0 => fly South
NORTH = 2
SOUTH = -2

# Note for vy:
# vy > 0 => fly East
# vy < 0 => fly West
EAST = 2
WEST = -2

# Note for vz: 
# vz < 0 => ascend
# vz > 0 => descend
UP = -0.5
DOWN = 0.5


# Square path using velocity
# print("SQUARE path using SET_POSITION_TARGET_LOCAL_NED and velocity parameters")

# print("Yaw 180 absolute (South)")
# condition_yaw(180)

# print("Velocity South & up")
send_ned_velocity(2, 1, UP, 5)
send_ned_velocity(2, 2, UP, 5)
send_ned_velocity(-1, 1, UP, 5)
send_ned_velocity(-2, 21, UP, 5)
# send_ned_velocity(0, 0, 0, 1)

# print("Yaw 270 absolute (West)")
# condition_yaw(270)

# print("Velocity West & down")
# send_ned_velocity(0, WEST, DOWN, DURATION)
# send_ned_velocity(0, 0, 0, 1)

# print("Yaw 0 absolute (North)")
# condition_yaw(0)

# print("Velocity North")
# send_ned_velocity(NORTH, 0, 0, DURATION)
# send_ned_velocity(0, 0, 0, 1)

# print("Yaw 90 absolute (East)")
# condition_yaw(90)

# print("Velocity East")
# send_ned_velocity(0, EAST, 0, DURATION)
# send_ned_velocity(0, 0, 0, 1)


# # Global
# print("Yaw 225 absolute")
# condition_yaw(225)

# print("Velocity South, West and Up")
# send_global_velocity(SOUTH, WEST, UP, DURATION)
# send_global_velocity(1, -2, 0, DURATION)
# send_global_velocity(0, 0, 0, 1)

# print("Yaw 90 relative (to previous yaw heading)")
# condition_yaw(90, relative=True)

# print("Velocity North, West and Down")
# send_global_velocity(NORTH, WEST, DOWN, DURATION)
# send_global_velocity(0, 0, 0, 1)

# print("Set new home location to current location")
# vehicle.home_location = vehicle.location.global_frame
# print("Get new home location")
# #This reloads the home location in DroneKit and GCSs
# cmds = vehicle.commands
# cmds.download()
# cmds.wait_ready()
# print(" Home Location: %s" % vehicle.home_location)


# print("Yaw 90 relative (to previous yaw heading)")
# condition_yaw(90, relative=True)

# print("Velocity North and East")
# send_global_velocity(NORTH, EAST, 0, DURATION)
# send_global_velocity(0, 0, 0, 1)

# print("Yaw 90 relative (to previous yaw heading)")
# condition_yaw(90, relative=True)

# print("Velocity South and East")
# send_global_velocity(SOUTH, EAST, 0, DURATION)
# send_global_velocity(0, 0, 0, 1)


# print("Setting RTL mode...")
# vehicle.mode = VehicleMode("RTL")


time.sleep(30)

# vehicle.remove_attribute_listener('location.global_relative_frame', location_relative_callback)
# vehicle.remove_attribute_listener('location.global_frame', location_global_callback)
# vehicle.remove_attribute_listener('attitude', attitude_callback)

#Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()