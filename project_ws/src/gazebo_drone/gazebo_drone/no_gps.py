#!/usr/bin/env python3 

import numpy as np # for turning the image array into numpy arrays

import cv2 # for detecting aruco markers using computer vision

import sys # in case we need to quit the program prematurely

import time # for helping with creating realistic frame rate

import cv_bridge # for the bridge between computer vision and ros2
from cv_bridge import CvBridge

from dronekit import connect, Vehicle

from pymavlink import mavutil

import pyquaternion

import math

from scipy.integrate import cumtrapz


################### VARIABLES SECTION ###################

# set up the vehicle with drone kit
drone_vehicle = connect("tcp:127.0.0.1:5763", wait_ready = True)
#drone_vehicle.parameters['PLND_ENABLED'] = 1
#drone_vehicle.parameters['PLND_ENABLED'] = 1
#drone_vehicle.parameters['PLND_ENABLED'] = 0
drone_vehicle.parameters['LAND_SPEED'] = 50 # in centimeters per second

takeoff_altitude = 1.5# at 18 meters I could see all the markers but uhhhhh
################### FUNCTIONS SECTION ###################

# we cannot use simple takeoff because we are not using gps, and that requires a GPS starting coordinate.
# we can instead command the drone to thrust until it reaches the target height (approximately)

# stabilize method to try to correct pitch and roll errors
def stabilize(desiredRoll, desiredPitch, desiredYaw,  desiredXDisplacement, currentXDisplacement, desiredYDisplacement, currentYDisplacement): # doesn't help with horizontal drift...
     # get the current roll and pitch
    roll = drone_vehicle.attitude.roll
    pitch = drone_vehicle.attitude.pitch
    yaw = drone_vehicle.attitude.yaw

    print("\n\tCurrent Roll, pitch, yaw:", roll, pitch, yaw)

    # we want roll and pitch to be the desired angle, so we try to correct them
    corrected_roll = desiredRoll - roll
    corrected_pitch = desiredPitch - pitch
    corrected_yaw = desiredYaw - yaw

    # now, we want to adjust the roll and pitch based on the desired velocity in the x and y axises
    # for takeoff, for example, we want the velocity in the x and y axises to be as close to zero as possible

    corrected_y = currentYDisplacement - desiredYDisplacement
    corrected_x = currentXDisplacement - desiredXDisplacement

    # we want a threshold for what to correct for, as well as how many degrees to shift based on the meters the drone has shifted
    threshold = 0.1
    correction_degrees = 10

    print("\tDesired X, current X", desiredXDisplacement, currentXDisplacement)
    print("\tDesired Y, current y", desiredYDisplacement, currentYDisplacement)


    # if the drift exceeds the threshold, correct it
    if abs(corrected_x) > threshold:
        print("\t Attempting to correct x-axis drift")
                                # np clip(value to be clipped, min value, max value)
        corrected_x_degrees = np.clip(corrected_x * -correction_degrees, -correction_degrees, correction_degrees)
        corrected_roll += np.deg2rad(corrected_x_degrees)



      # if the drift exceeds the threshold, correct it
    if abs(corrected_y) > threshold:
        print("\t Attempting to correct y-axis drift")
                                # np clip(value to be clipped, min value, max value)
        corrected_y_degrees = np.clip(corrected_y * -correction_degrees, -correction_degrees, correction_degrees)
        corrected_pitch += np.deg2rad(corrected_y_degrees)


    print('\tDesired Roll, pitch, yaw:', desiredRoll, desiredPitch, desiredYaw)

    print("\tCorrection Roll, pitch, yaw:", corrected_roll, corrected_pitch, corrected_yaw)
    

    # return proper quaternion to try to prevent drift
    return euler_to_quaternion(corrected_roll, corrected_pitch, corrected_yaw)

# enter angles in radians
def euler_to_quaternion(roll, pitch, yaw):
    # using the mathematic formula and np.cos and np.sin
    # in the order w, x, y, z

    sin_roll = np.sin(roll * 0.5)
    cos_roll = np.cos(roll * 0.5)

    sin_pitch = np.sin(pitch * 0.5)
    cos_pitch = np.cos(pitch * 0.5)

    sin_yaw = np.sin(yaw * 0.5)
    cos_yaw = np.cos(yaw * 0.5)

    qw = cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw
    qx = sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw
    qy = cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw
    qz = cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw

    print(f"\tquaternion: [{qw}, {qx}, {qy}, {qz}]")
    return [qw, qx, qy, qz]


# VARIABLES FOR CALCULATING DISPLACEMENT
x_accels = [0] # holds x accelerations
y_accels = [0] # holds y accelerations
times = [0] # holds seconds passed from the start time
start_time = time.time()
distance = 0
x_displacement = []
xydisplacement = []

@drone_vehicle.on_message('RAW_IMU') # define a listener for the RAW_IMU data
# recieve IMU data from ardupilot with mavlink
def get_imu(self, name, message):
    global times, x_accels, start_time
    # the accelerometer tells us the linear acceleration (change in velocity)
    #Accelerometer (x, y, z): xacc, message.yacc, message.zacc
    #Gyroscope (x, y, z): xgyro, ygyro, zgyro
    
    t = time.time() - start_time # assign seconds from start time to times list
    ax = message.xacc # assign x acceleration
    ay = message.yacc # assign y acceleration

    times.append(t)
    x_accels.append(ax)
    y_accels.append(ay)

# resetting between sending another thrust message
def set_vars_zero():
    global times, x_accels, y_accels, distance, x_displacement, y_displacement,  start_time
    times = [0]
    x_accels = [0]
    y_accels = [0]
    distance = 0 
    x_displacement = []
    y_displacement = []
    start_time = time.time()


# return displacement of drone based on acceleration values on the x axis
def calculate_displacement(x_accels, y_accels):
    if(len(times) == len(x_accels) == len(y_accels)):
        # calculate velocity
        x_vel = cumtrapz(np.array(x_accels), np.array(times), initial = 0)
        print('\n\tx_vel', x_vel[-1])

        y_vel = cumtrapz(np.array(y_accels), np.array(times), initial = 0)
        print('\n\ty_vel', y_vel[-1])

        #z_vel = cumtrapz(accels[2], time.time() - start_time, initial=0)
    
        # calculate displacement
        x_displacement = cumtrapz(np.array(x_vel), np.array(times),  initial = 0)
        y_displacement = cumtrapz(np.array(y_vel), np.array(times),  initial = 0)
        #z_displacement = cumtrapz(z_vel, time,  initial =  0)

        print('\t**x_displacement: ', x_displacement[-1])
        print('\t**y_displacement: ', y_displacement[-1])

        return [x_displacement[-1], y_displacement[-1]]


# send mavlink message telling the drone to thrust
def send_thrust_message(desired_q, thrust):
    # *****STABILIZATION
    # print('desired_q', desired_q)

    message = drone_vehicle.message_factory.set_attitude_target_encode(
        0,# time_boot_ms -- we don't use this
        1,# target system
        1,# target componenet
        0b00000111, # type mask              0b00000111 means ignore roll rate, pitch rate, and yaw rate
        desired_q, # quaternion -> {w, x, y, z} --> zero rotation is {1, 0, 0, 0} (causes vehicle to rotate north)
        0, # body roll rate not supported
        0, # body pitch rate not supported
        0, # body yaw rate not supported 
        thrust # thrust -- 50%
    )

    drone_vehicle.send_mavlink(message)

    

# function to initiate takeoff 
def takeoff():
    """
    # FOR NOW... Arming checks are disabled.

    # ensure drone is armable
    while drone_vehicle.is_armable != True:
        print('Waiting for the drone to become armable...')
        time.sleep(1)

    print("Drone now armable!")"""

    # set to guided_noGPS mode
    print("Setting mode to GUIDED_NOGPS")
    if drone_vehicle.mode != "GUIDED_NOGPS":
        drone_vehicle.mode = "GUIDED_NOGPS"
        while drone_vehicle.mode != "GUIDED_NOGPS":
            print('Waiting for the drone to be in GUIDED_NOGPS mode...')
            time.sleep(1)

        print("Drone now in GUIDED_NOGPS mode!")

    # arm the drone
    if drone_vehicle.armed != True:
        drone_vehicle.armed = True
        while drone_vehicle.armed != True:
            print('Arming the drone...')
            time.sleep(1)

        print("Drone now armed, ready for takeoff!")


    # takeoff message
    print('Taking off...')
 
    # while the drone's estimated altitude is approximately less than the takeoff_altitude
    while drone_vehicle.location.global_relative_frame.alt < takeoff_altitude * 0.95: # gets the height from barometer
        # print x displacement
        displacements = calculate_displacement(x_accels, y_accels) 

        # adjust based on displacement?

        # desired pitch, roll, and yaw all zero. desired x, current x. desired y, current y
        desired_q = stabilize(0, 0, 0, 0, displacements[0], 0, displacements[1]) 

        
        #print('\tdesired: q', [1,0,0,0])

        # send a thrust message to move upwards -- no pitch or roll
        send_thrust_message(desired_q, 0.6) # takes in thrust

        # print current height
        print('\tCurrent Height: ', drone_vehicle.location.global_relative_frame.alt)
        time.sleep(0.2)
    
    print('takeoff complete')
    while True:
        # print x displacement
            displacements = calculate_displacement(x_accels, y_accels) 

            # adjust based on displacement?

            # desiredRoll, desiredPitch, desiredYaw,  desiredXDisplacement, currentXDisplacement, desiredYDisplacement, currentYDisplacement
            desired_q = stabilize(0, 0, 0, 0, displacements[0], 0, displacements[1]) 

            
            #print('\tdesired: q', [1,0,0,0])

            # send a thrust message to move upwards -- no pitch or roll
            send_thrust_message(desired_q, 0.3) # takes in thrust

            # print current height
            print('\tCurrent Height: ', drone_vehicle.location.global_relative_frame.alt)
            time.sleep(0.2)
    
    
# function for carrying out flight path
def flight_path():
    print('\nInitiating flight path')
    target_distance = 3 # in meters

    set_vars_zero()

    # move forward:

    # small forward tilt --> moves forward
    roll_angle = 0
    pitch_angle = -(5 * math.pi) / 180  # convert degrees to radians
    yaw_angle = 0
    while True:
        # calculate the distance travelled
        displacements = calculate_displacement(x_accels, y_accels)
        x_distance = abs(displacements[0])

        # if the distance is greater than the target distance, change the thrust and break from the loop
        if x_distance > target_distance * 0.95:
            print('made it!')

            send_thrust_message([1,0,0,0], 0) # 0.5 thrust no tilt

            break

        q = euler_to_quaternion(roll_angle, pitch_angle, yaw_angle)
        print("\t quaternion: ", q)

        send_thrust_message(q, 0.5)

        time.sleep(0.5)


    target_distance = 14 # meters
    # move backward
    """set_vars_zero()

    print(times)
    print(x_accels)
    while True:

        distance = abs(calculate_displacement(x_accels))

        if distance > target_distance * 0.95:
            print('made it!')
            send_thrust_message([1,0,0,0], 0)
            break

        # small backward tilt
        pitch_angle = 15 * math.pi / 180  # convert degrees to radians
        roll_angle = 0

        # create separate quaternions
        roll_q = pyquaternion.Quaternion(axis=[1,0,0], angle=roll_angle)
        pitch_q = pyquaternion.Quaternion(axis=[0,1,0], angle=pitch_angle)

        q = roll_q * pitch_q

        #desired_q = stabilize(pitch_angle, roll_angle)  # use stabilize to adjust quaternion
        #print(desired_q)

        send_thrust_message(q, 0.5)

        time.sleep(0.5)"""

def testing():
    print('testing....')
     # arm the drone
    if drone_vehicle.armed != True:
        drone_vehicle.armed = True
        while drone_vehicle.armed != True:
            print('Arming the drone...')
            time.sleep(1)

        print("Drone now armed, ready for takeoff!")

    # set to guided_noGPS mode
    print("Setting mode to GUIDED_NOGPS")
    if drone_vehicle.mode != "GUIDED_NOGPS":
        drone_vehicle.mode = "GUIDED_NOGPS"
        while drone_vehicle.mode != "GUIDED_NOGPS":
            print('Waiting for the drone to be in GUIDED_NOGPS mode...')
            time.sleep(1)

        print("Drone now in GUIDED_NOGPS mode!")


    send_thrust_message([1,0,0,0], 0.5)


# main function
def main():
    print("Initiating takeoff!")

    # initiate takeoff
    takeoff()
    print("Takeoff completed.")

    #flight_path()
    print("Flight path completed.")
    #call land function here
    

# initiate main :)
if __name__ == '__main__': main()

