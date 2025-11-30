#!/usr/bin/env python3 

# make sure you make the script executable!
import rclpy # ros2 package

import sys # in case we need to quit the program prematurely

from dronekit import connect
from pymavlink import mavutil

import math

import time

import socket # for sending/recieving messages

from haversine import haversine, Unit # for calculating the distance between two lattitude and longitude coordinates

from rclpy.node import Node

import numpy as np # for turing the image array into numpy arrays

import cv2 # for detecting aruco markers using computer vision

from sensor_msgs.msg import Image # message type for our topic we are publishing/subscribing (new_image)

from cv_bridge import CvBridge # for the bridge between computer vision and ros2

import threading


################### VARIABLES FOR ARUCO DETECTION ###################

# specify aruco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL) 

# specify aruco parameters
aruco_parameters = cv2.aruco.DetectorParameters()

# aruco detector
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_parameters)

# camera calibration (camera matrix and distortion coefficients)
# find by running our challenge_1 ros gazebo world and viewing ros2 topic list /camera/camera_image

"""
ros2 topic echo /camera/camera_info
distortion and 

d:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
k:
- 1061.6538553425996
- 0.0
- 640.5
- 0.0
- 1061.6538553425996
- 360.5
- 0.0
- 0.0
- 1.0
"""

camera_matrix = [[1061.6538553425996, 0.0, 640.5],[0.0, 1061.6538553425996, 360.5],[0.0, 0.0, 1.0]]
np_camera_matrix = np.array(camera_matrix) # convert to numpy array

distortion_co = [0.0, 0.0, 0.0, 0.0, 0.0]
np_distortion_co  = np.array(distortion_co) # convert to numpy array


target_ID = 72 #129 # target aruco marker ID

threshold = 0 

marker_length = 0.3048 # 1 ft

detected = False

aruco_points = np.array([
    [-marker_length/2, marker_length/2, 0], 
    [marker_length/2, marker_length/2, 0],
    [marker_length/2, -marker_length/2, 0],
    [-marker_length/2, -marker_length/2, 0],
    ]
    )

horizontal_res = 1280 # width in sdf file
vertical_res = 720 # height in sdf file

horizontal_fov = 1.085 # from sdf file
vertical_fov = 48.8 * (math.pi / 180)

global angle
angle = 0

detected_markers = {}

# connect the primary vehicle to the SITL
print('Connecting to secondary vehicle...') # with I1 tag

# set up the vehicle with drone kit
secondary_vehicle = connect("udp:127.0.0.1:14560", wait_ready = True) # was tcp:127.0.0.1:5763
secondary_vehicle.parameters['PLND_ENABLED'] = 1 # percision landing enabled
secondary_vehicle.parameters['PLND_TYPE'] = 1 # target position source from MAVLINK
secondary_vehicle.parameters['PLND_EST_TYPE'] = 0 # raw sensor, not kalman filter
secondary_vehicle.parameters['LAND_SPEED'] = 50 # in centimeters per second

print('Connected to secondary vehicle!\n')


################### PUB/SUB SECTION ###################


# publisher to publish the camera feed (type, topic, queue size)
class ImageSub(Node):
    def __init__(self):
        # create the publisher
        super().__init__('image_sub_2')

         # create the subscriber
        self.subscriber = self.create_subscription(Image, '/camera_2/camera/image_raw', self.callback, 10)
        self.subscriber


    def callback(self, message):
        bridge = CvBridge()
        
        # convert ROS Image message to OpenCV iamge
        np_data = bridge.imgmsg_to_cv2(message, desired_encoding='rgb8') # convert ROS Image message to openCV Image (which is a numpy array)
        # get image and convert to grayscale for better accuracy
        gray_image = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)
    
        # detect aruco markers in frame
        # get the corners of the aruco markers and the IDs that it returned with detectMarkers function
        (corners, returned_ids, rejected) = aruco_detector.detectMarkers(gray_image)

        if returned_ids is not None: # if an ID is found

            if returned_ids[0] == target_ID: # if the ID found is the target ID     

                global detected 
                detected = True

                global angle

                if int(returned_ids[0]) not in detected_markers:
                    detected_markers[int(returned_ids[0])] = 1
                    print("\nDetected Target Marker. Beginning Precision Landing.")

                # get the x and y distances from aruco's center
                returned, rvec, tvec = cv2.solvePnP(aruco_points, corners[0], np_camera_matrix, np_distortion_co)

                # calculate Xerror/distance between camera and aruco (in centimeters)
                X = round(tvec[0][0], 2)
                Y = round(tvec[1][0], 2)
                Z = round(tvec[2][0], 2)

                threshold = 0.08
                speed = 0.1
                descending = 0.1

                # perform precision landing
                # move diagonally toward aruco
                if abs(Y) > threshold and abs(X) > threshold:
                    #print("moving... diagonal")
                    velocity_x = speed if Y < 0 else -speed
                    velocity_y = -speed if X < 0 else speed
                    send_velocity(velocity_x, velocity_y, descending)
                
                # move up/down toward aruco
                elif abs(Y) > threshold and abs(X) < threshold:
                    #print("moving... up/down")
                    velocity = speed if Y < 0 else -speed
                    send_velocity(velocity, 0, .1)
                
                # move left/right toward aruco
                elif abs(X) > threshold and abs(Y) < threshold:
                    #print("moving... left/right")
                    velocity = -speed if X < 0 else speed
                    send_velocity(0, velocity, descending)
                else:
                    send_velocity(0, 0, descending)

                time.sleep(0.5) # buffer time
                
        else:
            if detected and secondary_vehicle.location.global_relative_frame.alt <= 0.04:
                print("\nSecondary vehicle landed.")
                sys.exit()

                rclpy.shutdown()
            elif detected and secondary_vehicle.location.global_relative_frame.alt >= 0.04:
                # keep descending
                send_velocity(0, 0, 0.1)
                


################### FUNCTIONS SECTION ###################

# secondary drone recieve message over serial port
def receive_message():

    # Create a UDP socket
    sock = socket.socket(socket.AF_INET,  # internet
                        socket.SOCK_DGRAM) # udp

    # Send the message
    sock.bind(('127.0.0.1', 14650))

    while True:
        data, address = sock.recvfrom(1024) # buffer size of 1024

        data_decoded = data.decode('utf-8')
        data_decoded = data_decoded.split(", ")

        x = float(data_decoded[0])
        y = float(data_decoded[1])
        print(f"Received message from primary drone: {x} m, {y} m")

        return x, y
    

# function to initiate takeoff
def takeoff():
    takeoff_altitude = 3 

    # ensure drone is armable
    print('\nWaiting for the drone to become armable...')
    while secondary_vehicle.is_armable != True:
        time.sleep(1)

    print("Secondary drone now armable!\n")

    # set to guided mode
    if secondary_vehicle.mode != "GUIDED":
        secondary_vehicle.mode = "GUIDED"
        print('Waiting for the drone to be in GUIDED mode...')
        while secondary_vehicle.mode != "GUIDED":
            time.sleep(1)

        print("Secondary drone now in GUIDED mode!")

     # arm the drone
    if secondary_vehicle.armed != True:
        secondary_vehicle.armed = True
        print('\nArming the drone...')
        while secondary_vehicle.armed != True:
            time.sleep(1)

        print("Secondary drone now armed, ready for takeoff!")

    secondary_vehicle.simple_takeoff(takeoff_altitude)

    print("Taking off!")

    while True:
        time.sleep(1)
        print(f'\tRising in altitude, currently at {secondary_vehicle.location.global_relative_frame.alt:.2f} m')

        # if the target height is about 95% accurate
        if secondary_vehicle.location.global_relative_frame.alt >= .95 * takeoff_altitude:
            break

    time.sleep(1)
    print("Target altitude reached, secondary vehicle takeoff complete!")



def send_velocity(vel_x, vel_y, vel_z):
    message = secondary_vehicle.message_factory.set_position_target_local_ned_encode(
        0,# time_boot_ms -- we don't use this
        0,# target system
        0,# target componenet
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,# frame, MAV_FRAME_BODY_NED is for no rotation, MAV_FRAME_LOCAL_NED is for rotation
        0b0000111111000111,# type_mask
        0,0,0,# x, y, z positions
        vel_x, # +x --> north, -x --> south
        vel_y, # +y --> east, -x --> west
        vel_z, # +z --> descend, -z --> ascend
        0,0,0,# x, y, z acceleration
        0,# yaw
        0 # yaw_rate
    )

    secondary_vehicle.send_mavlink(message)
    

# return drone's current lat and long
def get_current_position():
    return (secondary_vehicle.location.global_relative_frame.lat, secondary_vehicle.location.global_relative_frame.lon)


# command the drone to fly at a particular velocity until target distance is reached
def control_drone_velocity(start_position, target_distance, vel_x, vel_y, vel_z):
    current_location = get_current_position()

    distance_travelled = 0
    
    # send command to vehicle every second (1 hz) until we reach desired distance
    while (distance_travelled < target_distance * 0.95 and detected == False): # threshold so drone stops on time
        current_location = get_current_position()
        distance_travelled = (haversine(start_position, current_location, Unit.KILOMETERS)) * 1000
        
        #print(f"\tDistance from target: {(target_distance - distance_travelled):.2f}")
        
        send_velocity(vel_x, vel_y, vel_z)
        time.sleep(0.1)
        

    send_velocity(0,0,0) # stop the drone when target destination is reached


# move towards the aruco marker based on the primary drone's message
def move_to_aruco(x, y):
    start_position = get_current_position() # start position lat and lon
    speed = 0.5
    # move forward to aruco
    print(f"\nMoving forward to aruco {x:2f} m")
    start_position = get_current_position()
    control_drone_velocity(start_position, 1, speed, 0, 0)

    start_position = get_current_position()
    velocity_x = -speed if x < 0 else speed
    control_drone_velocity(start_position, abs(x), velocity_x, 0, 0)

    print(f"Moving right/left to aruco {y:2f} m")
    # move left/right to aruco
    start_position = get_current_position()
    velocity_y = -speed if y < 0 else speed
    control_drone_velocity(start_position, abs(y + 1), 0, velocity_y, 0)



def precision_land():
    rclpy.init(args=None)
    global image_sub
    image_sub = ImageSub()
    rclpy.spin(image_sub)


def turn_angle(angle):
    print("angle to turn in degrees", angle)

    # turn drone that angle
    secondary_vehicle.message_factory.command_long_send(
        0, # target system
        0, # target_component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, # command (also equal to 400)
        0, # confirmation
        angle, # param 1 --> yaw in degrees
        5, # param 2 --> yaw in speed degree/s (30 degrees per second)
        1, # param 3 --> direction, -1 is counterclockwise and 1 is clockwise
        0, # param 4 --> relative offset 1, absolute angle 0
        0, # not used --> params 5 - 7 are not used.
        0, # not used
        0, # not used
    )

    

def main():
    print('Waiting on message from primary drone...')
    # receive message, then takeoff
    distances = receive_message()    

    x_to_travel = distances[0]
    y_to_travel = distances[1]

    takeoff()

    # use threading to run the imagesub node and the move_to_arcuo function
    global thread2
    thread1 = threading.Thread(target=precision_land)
    thread2 = threading.Thread(target=move_to_aruco, args=(x_to_travel, y_to_travel))

    thread1.start()
    thread2.start()
    


# initiate main
if __name__ == '__main__': main()