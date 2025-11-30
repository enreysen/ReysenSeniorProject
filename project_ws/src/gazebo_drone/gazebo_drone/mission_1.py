#!/usr/bin/env python3 

"""

Challenge 1 Flight Path Logic

Challenge 1:
    Autonomously search the field with a UAV to find the target aruco marker
    Upon finding the target marker, output that it was found.
    Then, fly off the field and land.


    If the Aruco marker was not found, the flight path will finish, output the marker was not found, and then move off the field and land.

    To accomplish aruco marker detection, I created a subscriber node to retrieve data from the drone's camera (which is published with a ros2 publisher node),
    and used theOpenCV library to detect the marker.

    
"""

# aruco tracking and primary vehicle flight path in one file
# this is because I needed to use a function 'upon_detection' once the aruco marker was found

import sys

from dronekit import connect

from pymavlink import mavutil

import time

from haversine import haversine, Unit # for calculating the distance between two lattitude and longitude coordinates

import rclpy # ros2 package

from rclpy.node import Node # for creating publisher and subscriber nodes

import numpy as np # for turing the image array into numpy arrays

import cv2 # for detecting aruco markers using computer vision

from sensor_msgs.msg import Image # message type for our topic we are publishing/subscribing (new_image)

from cv_bridge import CvBridge # for the bridge between computer vision and ros2


################### VARIABLES FOR ARUCO DETECTION ###################

# specify aruco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL) # the aruco markers used were created with the Original dictionary

# specify aruco parameters
aruco_parameters = cv2.aruco.DetectorParameters()

# aruco detector
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_parameters)

# camera calibration (camera matrix and distortion coefficients)
# find by running challenge_1 ros-gazebo world and running ros2 topic list /camera/camera_image in a terminal

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

marker_length = 0.3048 # 1 ft

aruco_points = np.array([
    [-marker_length/2, marker_length/2, 0], 
    [marker_length/2, marker_length/2, 0],
    [marker_length/2, -marker_length/2, 0],
    [-marker_length/2, -marker_length/2, 0],
    ]
    )

# to track the markers that have been detected
detected_markers = {}


################### VARIABLES FOR FLIGHT PATH ###################


# connect the drone vehicle to the SITL
print('Connecting to drone...')

# set up the drone vehicle with drone kit
drone_vehicle = connect("udp:127.0.0.1:14550", wait_ready = True) # connected through UDP
drone_vehicle.parameters['LAND_SPEED'] = 50 # land speed in centimeters per second

print('Connected to drone!\n')

global sub_node # global variable for spinning the subscriber node

# tracks how far the drone has travelled. I track this so I know how much to move off the field
x_travelled = 0
y_travelled = 0


################### SUBSCRIBER NODE SECTION ###################


# publisher to publish the camera feed (type, topic, queue size)
class ImageSub(Node):
    def __init__(self):
        # create the publisher
        super().__init__('image_sub')

         # create the subscriber
        self.subscriber = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
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
            id = int(returned_ids[0])

            if id == target_ID: # if the ID found is the target ID     
                self.get_logger().info(f'********TARGET MARKER SPOTTED: {id}')
                
                # once the target marker is found, we want to get off the field and land
                upon_detection()
                rclpy.shutdown()

            else: 
                # I add the id to the dictionary so that the below messaage is not printed to the screen more than once.
                if id not in detected_markers: # if it has not already been found, output its id
                    detected_markers[id] = 1
                    self.get_logger().info(f"********Marker detected, but not target marker: {id}")



################### FUNCTIONS SECTION ###################

# function to initiate takeoff
def takeoff():
    # altitude to take off
    takeoff_altitude = 3 

    # ensure drone is armable
    print('Waiting for the drone to become armable...')

    while drone_vehicle.is_armable != True:
        time.sleep(1)

    print("Drone now armable!\n")


    print('Waiting for the drone to be in GUIDED mode...')
    # set to guided mode
    if drone_vehicle.mode != "GUIDED":
        drone_vehicle.mode = "GUIDED"
        while drone_vehicle.mode != "GUIDED":
            
            time.sleep(1)

        print("Drone now in GUIDED mode!")

    # arm the drone
    if drone_vehicle.armed != True:
        drone_vehicle.armed = True
        print('\nArming the drone...')
        while drone_vehicle.armed != True:
            time.sleep(1)

        print("Drone now armed, ready for takeoff!")

    # takeoff
    drone_vehicle.simple_takeoff(takeoff_altitude)
    # primary taking off

    print("Taking off!")
    while True:
        print(f'\tRising in altitude, currently at {abs(drone_vehicle.location.global_relative_frame.alt):.2f} m') # abs() because sometimes drone outputs negative altitude

        # if the target height is about 95% accurate, break out of the loop
        if drone_vehicle.location.global_relative_frame.alt >= .95 * takeoff_altitude:
            break

        time.sleep(1)

    time.sleep(1)
    print("Target altitude reached, drone takeoff complete!!")
    

# return drone's current lat aand long
def get_current_position():
    return (drone_vehicle.location.global_relative_frame.lat, drone_vehicle.location.global_relative_frame.lon)


# send a mavlink messaage to ardupilot that holds velocity commands for the drone
def send_velocity(vel_x, vel_y, vel_z):
    global x_travelled
    global y_travelled

    # add to x_travelled and and y_travelled based on the velocity it travels every 0.1 seconds
    x_travelled += vel_x / 10 # we send this message every .1 second. if drone is flying 0.5 m/s for example, we need to 1/10 of it for our estimation
    y_travelled += vel_y / 10 

    # create the velocity message
    message = drone_vehicle.message_factory.set_position_target_local_ned_encode(
        0,# time_boot_ms 
        0,# target system
        0,# target componenet
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,# frame, MAV_FRAME_BODY_NED is for no rotation, MAV_FRAME_LOCAL_NED is for rotation
        0b0000111111000111,# type_mask
        0,0,0,# x, y, z positions
        vel_x, # +x --> north, -x --> south
        vel_y, # +y --> east, -x --> west
        vel_z, # +z --> descend, -z --> ascend
        0,0,0, # x, y, z acceleration
        0,# yaw
        0 # yaw_rate
    )

    # send the messaage
    drone_vehicle.send_mavlink(message)
    

# command the drone to fly at a particular velocity until target distance is reached
# stop sending the command when it reaches around the target distance
def control_drone_velocity(start_position, target_distance, vel_x, vel_y, vel_z):
    current_location = (drone_vehicle.location.global_relative_frame.lat, drone_vehicle.location.global_relative_frame.lon)

    distance_travelled = 0

    print_time = time.time()
    
    # send command to vehicle every second (1 hz) until we reach desired distance
    while (distance_travelled < target_distance * 0.95): # threshold of 0.95, does not need to be exactly the target distance
        current_location = get_current_position()

        # with the haversine formula, get the distance between where the drone started and it's current location
        # this calculates the distance travelled from the start position
        distance_travelled = (haversine(start_position, current_location, Unit.KILOMETERS)) * 1000


        # only print "distance from target" around every 1 second
        if (time.time() - print_time > 1):
            #print(f"\tDistance from target: {(target_distance - distance_travelled):.2f}")
            print_time = 0

        rclpy.spin_once(sub_node) # spin the aruco detection node while moving
        
        # send the velocity command every 0.1 seconds
        send_velocity(vel_x, vel_y, vel_z)
        time.sleep(0.1)

    send_velocity(0,0,0) # stop the drone when target destination is reached


# send drone to specific points within the field
def flight_path():
    print("EXECUTING FLIGHT PATH...")
    time.sleep(1)

    # I used the 'spiral' flight path here, as it was the fastest of the three I creaated (diagonal, spirl, and lawnmower)
    # I found that to fly in a stable manner, I needed to travel no more than 0.5 m/s in any axis
    speed = 0.5

    # move up 
    print("Moving ~8 meters up.")
    control_drone_velocity(get_current_position(), 8, speed, 0, 0)

    print("Moving ~6 meters right.")
    # move right
    control_drone_velocity(get_current_position(), 5.8, 0, speed, 0)

    print("Moving ~ 7 meters down.")
    # move down
    control_drone_velocity(get_current_position(), 7, -speed, 0, 0)

    print("Moving ~ 3 meters left.")
    # move left
    control_drone_velocity(get_current_position(), 2.8, 0, -speed, 0)

    print("Moving ~ 5 meters up.")
    # move up
    control_drone_velocity(get_current_position(), 5.7, speed, 0, 0)
    
    print("Drone finished flight path, moving off the field")

    # move off field
    target_distance = 3
    control_drone_velocity(get_current_position(), target_distance, speed, 0, 0)


# function to land the drone
def land():
    print("INITIATING LANDING")
    # set to land mode
    if drone_vehicle.mode != "LAND":
        drone_vehicle.mode = "LAND"
        while drone_vehicle.mode != "LAND":
            print('Waiting for the drone to be in LAND mode...')
            time.sleep(1)

        print("The drone is now in LAND mode.")

        print("Initating landing!")
        while drone_vehicle.location.global_relative_frame.alt > 0.03: # ONce on the ground, the altitude of the drone slowly falls from 0.03 to 0.
            print("\tLanding..")
            time.sleep(1)

        print("Drone sucessfully landed!")
        
        sys.exit() # exit the script


# upon detecting the aruco marker, move off the field
def upon_detection():
    global x_travelled

    print("\nAruco Marker was detected. Moving off the field.")

    # go to edge of field
    print(f'Moving off the field.... Flying {(9 - x_travelled):.2f} m forward')
    start_position = (drone_vehicle.location.global_relative_frame.lat, drone_vehicle.location.global_relative_frame.lon) # start position lat and lon
    distance_travelled = 0
    target_distance = 9 - x_travelled
    
    # send command to vehicle every second (1 hz) until we reach desired distance
    while (distance_travelled < target_distance * 0.95): # threshold so drone stops on time
        current_location = (drone_vehicle.location.global_relative_frame.lat, drone_vehicle.location.global_relative_frame.lon)
        distance_travelled = (haversine(start_position, current_location, Unit.KILOMETERS)) * 1000

        send_velocity(0.5, 0, 0) # move 0.5 m/s in the x-axis
        time.sleep(0.1) # send every .1 seconds

    send_velocity(0,0,0) # stop the drone when target destination is reached

    print("Travelled off the field.")

    # land
    land()


# main function --> initilize rclpy for subscriber node, create subscriber node, takeoff, flight path, and land if no aruco marker was found
def main(args = None):
    global sub_node # set pub sub to global variable. I had to do this so I could spin the node in other functions

    # initalize rclpy
    rclpy.init(args=args)
    sub_node = ImageSub() # create the subscriber node

    takeoff()
    time.sleep(1)

    flight_path()

    # if the full flight path was completed, then the target marker was detected
    land()
    print("Target Aruco marker not detected :(")


# initiate main
if __name__ == '__main__': main()