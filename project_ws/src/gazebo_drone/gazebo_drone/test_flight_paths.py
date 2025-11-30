#!/usr/bin/env python3 

# aruco tracking and test vehicle flight path in one
# this is because I needed to use a function 'upon_detection' once the aruco marker was found
import sys
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil

import time

from haversine import haversine, Unit # for calculating the distance between two lattitude and longitude coordinates

import socket # for sending/recieving messages

import rclpy # ros2 package

from rclpy.node import Node # for creating publisher and subscriber nodes

import numpy as np # for turing the image array into numpy arrays

import cv2 # for detecting aruco markers using computer vision

from sensor_msgs.msg import Image # message type for our topic we are publishing/subscribing (new_image)

import cv_bridge # for the bridge between computer vision and ros2
from cv_bridge import CvBridge



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

aruco_points = np.array([
    [-marker_length/2, marker_length/2, 0], 
    [marker_length/2, marker_length/2, 0],
    [marker_length/2, -marker_length/2, 0],
    [-marker_length/2, -marker_length/2, 0],
    ]
    )

################### PUB/SUB SECTION ###################

# publisher to publish the camera feed (type, topic, queue size)
class ImagePubSub(Node):
    def __init__(self):
        # create the publisher
        super().__init__('image_pub_sub')
        self.publisher = self.create_publisher(Image, '/camera/new_image', 10)

        # ensure the frame rate is not too fast *****
        # publish data every .1 second for a frame rate of 10 per second
        duration = 1#0.1
        self.timer = self.create_timer(duration, self.timer_callback)

         # create the subscriber
        self.subscriber = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.subscriber

    def timer_callback(self):  
        mssg = Image()          
        #self.get_logger().info('Publishing data to /camera/new_image')
        self.publisher.publish(mssg) # publish the message


    def callback(self, message):
        global id_dict
        #self.get_logger().info(f'Retrieving camera/image_raw data')
        bridge = CvBridge()
        
        # convert ROS Image message to OpenCV iamge
        np_data = bridge.imgmsg_to_cv2(message, desired_encoding='rgb8') # convert ROS Image message to openCV Image (which is a numpy array)
        # get image and convert to grayscale for better accuracy
        gray_image = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)
    
        # detect aruco markers in frame
        # get the corners of the aruco markers and the IDs that it returned with detectMarkers function
        (corners, returned_ids, rejected) = aruco_detector.detectMarkers(gray_image)

        if returned_ids is not None: # if an ID is found
            if int(returned_ids[0]) not in id_dict:
                print("ID found", returned_ids[0])
                id_dict[int(returned_ids[0])] = 1



# connect the test vehicle to the SITL
print('Connecting to test vehicle...') # with I1 tag

# set up the vehicle with drone kit
drone_vehicle = connect("udp:127.0.0.1:14550", wait_ready = True) # was tcp:127.0.0.1:5763
drone_vehicle.parameters['PLND_ENABLED'] = 1
drone_vehicle.parameters['PLND_ENABLED'] = 1
drone_vehicle.parameters['PLND_ENABLED'] = 0
drone_vehicle.parameters['LAND_SPEED'] = 50 # in centimeters per second

print('Connected to test vehicle!\n')

takeoff_altitude = 0.5 # at 18 meters I could see all the markers but uhhhhh -- was at 2

x_travelled = 0

y_travelled = 0

global pub_sub

global detected
detected = False

id_dict = {}

################### FUNCTIONS SECTION ###################


# function to initiate takeoff
def takeoff():
    # ensure drone is armable
    print('Preparing for takeoff...')

    while drone_vehicle.is_armable != True:
        time.sleep(1)

    # set to guided mode
    if drone_vehicle.mode != "GUIDED":
        drone_vehicle.mode = "GUIDED"
        while drone_vehicle.mode != "GUIDED":
            
            time.sleep(1)

    # arm the drone
    if drone_vehicle.armed != True:
        drone_vehicle.armed = True
        print('Arming the drone...')
        while drone_vehicle.armed != True:
            time.sleep(1)

    # takeoff
    drone_vehicle.simple_takeoff(takeoff_altitude)
    # primary taking off

    print("Taking off!")
    while True:
        time.sleep(1)
        # if the target height is about 95% accurate
        if drone_vehicle.location.global_relative_frame.alt >= .95 * takeoff_altitude:
            break

    time.sleep(1)
    print("Target altitude reached, primary drone takeoff complete!!")
    

# we are going to try to direct the drone without gps --> but for now we use this function
def send_velocity(vel_x, vel_y, vel_z):
    global x_travelled
    global y_travelled
    x_travelled += vel_x / 10 # we send this message every .1 second. if drone is flying 0.5 m/s for example, we need to 1/10 of it for our estimation
    y_travelled += vel_y / 10 
    message = drone_vehicle.message_factory.set_position_target_local_ned_encode(
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

    drone_vehicle.send_mavlink(message)
    

# command the drone to fly at a particular velocity until target distance is reached
def control_drone_velocity(start_position, target_distance, vel_x, vel_y, vel_z):
    current_location = (drone_vehicle.location.global_relative_frame.lat, drone_vehicle.location.global_relative_frame.lon)

    distance_travelled = 0
    print_time = time.time()
    
    # send command to vehicle every second (1 hz) until we reach desired distance
    while (distance_travelled < target_distance * 0.95): # threshold so drone stops on time
        current_location = (drone_vehicle.location.global_relative_frame.lat, drone_vehicle.location.global_relative_frame.lon)
        distance_travelled = (haversine(start_position, current_location, Unit.KILOMETERS)) * 1000


        if (time.time() - print_time > 1):
            print(f"\tDistance from target: {target_distance - distance_travelled}")
            print_time = 0

        
        rclpy.spin_once(pub_sub) # spin only once.
        time.sleep(0.1)
        
        send_velocity(vel_x, vel_y, vel_z)


    send_velocity(0,0,0) # stop the drone when target destination is reached


# send drone to specific points within the field
def flight_path_1():
    print("INITIATING FLIGHT PATH -- DIAGONAL")
    
    time.sleep(1)

    start_position = (drone_vehicle.location.global_relative_frame.lat, drone_vehicle.location.global_relative_frame.lon) # start position lat and lon
    target_distance = 2.7
    control_drone_velocity(start_position, target_distance, 0.35, -0.12, 0)

    start_position = (drone_vehicle.location.global_relative_frame.lat, drone_vehicle.location.global_relative_frame.lon) # start position lat and lon
    target_distance = 1.5
    control_drone_velocity(start_position, target_distance, 0.5, 0, 0)

    start_position = (drone_vehicle.location.global_relative_frame.lat, drone_vehicle.location.global_relative_frame.lon) # start position lat and lon
    target_distance = 5.4
    control_drone_velocity(start_position, target_distance, -0.30, 0.35, 0)

    # move right
    start_position = (drone_vehicle.location.global_relative_frame.lat, drone_vehicle.location.global_relative_frame.lon) # start position lat and lon
    target_distance = 2.8
    control_drone_velocity(start_position, target_distance, 0, 0.5, 0)

    # # move up
    start_position = (drone_vehicle.location.global_relative_frame.lat, drone_vehicle.location.global_relative_frame.lon) # start position lat and lon
    target_distance = 1.3
    control_drone_velocity(start_position, target_distance, 0.5, 0, 0)

    # move up - left
    start_position = (drone_vehicle.location.global_relative_frame.lat, drone_vehicle.location.global_relative_frame.lon) # start position lat and lon
    target_distance = 8
    control_drone_velocity(start_position, target_distance, 0.20, -0.35, 0)

    # move up
    start_position = (drone_vehicle.location.global_relative_frame.lat, drone_vehicle.location.global_relative_frame.lon) # start position lat and lon
    target_distance = 1.5
    control_drone_velocity(start_position, target_distance, 0.5, 0, 0)

    # move down - right
    start_position = (drone_vehicle.location.global_relative_frame.lat, drone_vehicle.location.global_relative_frame.lon) # start position lat and lon
    target_distance = 8
    control_drone_velocity(start_position, target_distance, -0.20, 0.35, 0)

    # move up
    start_position = (drone_vehicle.location.global_relative_frame.lat, drone_vehicle.location.global_relative_frame.lon) # start position lat and lon
    target_distance = 2.8
    control_drone_velocity(start_position, target_distance, 0.5, 0, 0)

    # move up - left
    start_position = (drone_vehicle.location.global_relative_frame.lat, drone_vehicle.location.global_relative_frame.lon) # start position lat and lon
    target_distance = 5
    control_drone_velocity(start_position, target_distance, 0.10, -0.35, 0)

    # move up
    start_position = (drone_vehicle.location.global_relative_frame.lat, drone_vehicle.location.global_relative_frame.lon) # start position lat and lon
    target_distance = 1
    control_drone_velocity(start_position, target_distance, 0.5, 0, 0)

    # move down - right
    start_position = (drone_vehicle.location.global_relative_frame.lat, drone_vehicle.location.global_relative_frame.lon) # start position lat and lon
    target_distance = 8
    control_drone_velocity(start_position, target_distance, -0.10, 0.35, 0)

    print('\t done')
    
def flight_path_2():
    print("INITIATING FLIGHT PATH -- SPIRAL")
    speed = 0.5
    time.sleep(2)
    # move up ~
    start_position = (drone_vehicle.location.global_relative_frame.lat, drone_vehicle.location.global_relative_frame.lon) # start position lat and lon
    target_distance = 8
    control_drone_velocity(start_position, target_distance, speed, 0, 0)

    # move right
    start_position = (drone_vehicle.location.global_relative_frame.lat, drone_vehicle.location.global_relative_frame.lon) # start position lat and lon
    target_distance = 5.8
    control_drone_velocity(start_position, target_distance, 0, speed, 0)

    # move down
    start_position = (drone_vehicle.location.global_relative_frame.lat, drone_vehicle.location.global_relative_frame.lon) # start position lat and lon
    target_distance = 7
    control_drone_velocity(start_position, target_distance, -speed, 0, 0)

    # move left
    start_position = (drone_vehicle.location.global_relative_frame.lat, drone_vehicle.location.global_relative_frame.lon) # start position lat and lon
    target_distance = 2.8
    control_drone_velocity(start_position, target_distance, 0, -speed, 0)

    # move up
    start_position = (drone_vehicle.location.global_relative_frame.lat, drone_vehicle.location.global_relative_frame.lon) # start position lat and lon
    target_distance = 5.7
    control_drone_velocity(start_position, target_distance, speed, 0, 0)
    
    print("Drone finished flight path, moving off the field")

    # move off field
    start_position = (drone_vehicle.location.global_relative_frame.lat, drone_vehicle.location.global_relative_frame.lon) # start position lat and lon
    target_distance = 3
    control_drone_velocity(start_position, target_distance, speed, 0, 0)


def flight_path_3():
    print("INITIATING FLIGHT PATH -- LAWN MOWER")
    time.sleep(1)

    # move up ~8 m
    start_position = (drone_vehicle.location.global_relative_frame.lat, drone_vehicle.location.global_relative_frame.lon) # start position lat and lon
    control_drone_velocity(start_position, 8, 0.5, 0, 0)

    # move to the right ~3 meters
    start_position = (drone_vehicle.location.global_relative_frame.lat, drone_vehicle.location.global_relative_frame.lon) # start position lat and lon
    control_drone_velocity(start_position, 2.8, 0, 0.5, 0)

    # move down ~8 m
    start_position = (drone_vehicle.location.global_relative_frame.lat, drone_vehicle.location.global_relative_frame.lon) # start position lat and lon
    control_drone_velocity(start_position, 8, -0.5, 0, 0)

    # move right ~3 m
    start_position = (drone_vehicle.location.global_relative_frame.lat, drone_vehicle.location.global_relative_frame.lon) # start position lat and lon
    control_drone_velocity(start_position, 2.8, 0, 0.5, 0)

    # move up ~9.5 m
    start_position = (drone_vehicle.location.global_relative_frame.lat, drone_vehicle.location.global_relative_frame.lon) # start position lat and lon
    control_drone_velocity(start_position, 9.5, 0.5, 0, 0)

def land():
    print("INITIATING LANDING")
    time.sleep(2)
    # set to land mode
    if drone_vehicle.mode != "LAND":
        drone_vehicle.mode = "LAND"
        while drone_vehicle.mode != "LAND":
            time.sleep(1)

        while drone_vehicle.location.global_relative_frame.alt > 0.07:
            print("\tLanding..")
            time.sleep(1)

        print("Primary drone sucessfully landed!")


def main(args = None):
    global id_dict
    global pub_sub
    # initalize rclpyp
    rclpy.init(args=args)
    pub_sub = ImagePubSub() # create the pubsub

    takeoff()
    time.sleep(1)
    flight_path_1()
    #land()

    print("Found IDS: ")
    for key in id_dict.keys():
        print(key)

    print("\n Undetected IDS:")
    for i in range(1, 30):
        if i not in id_dict.keys():
            print("NOT FOUND:", i)

main()