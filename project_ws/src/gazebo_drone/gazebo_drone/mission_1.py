#!/usr/bin/env python3 

"""

Challenge 1 Flight Path Logic

Challenge 1:
    Autonomously search the field with a UAV to find the target aruco marker
    Upon finding the target marker, output that it was found.
    Then, fly off the field and land.

    If the Aruco marker was not found, the flight path will finish, output the marker was not found, and then move off the field and land.

    To accomplish aruco marker detection, I created a subscriber node to retrieve data from the drone's camera (which is published with a ros2 publisher node),
    and used the OpenCV library to detect the marker.

    
"""

# aruco tracking and primary vehicle flight path in one file
# this is because I needed to use a function 'upon_detection' once the aruco marker was found

import sys # for exiting the script

from dronekit import connect # the dronekit API assists with communicating with the ArduPilot flight controller firmware

from pymavlink import mavutil # the paymavlink API assists with creating custom MAVLink messages to send to ArduPIlot

import time # when sending commands to ArduPilot, spinning a ROS 2 node, etc., adding buffer time can be help with ensuring not to overload the system

from haversine import haversine, Unit # for calculating the distance between two lattitude and longitude coordinates

import rclpy # ROS 2 python package

from rclpy.node import Node # for creating publisher and subscriber nodes

import numpy as np # for assisting with image processing

import cv2 # for detecting aruco markers using computer vision

from sensor_msgs.msg import Image # use the Image ROS message type for our topic we are publishing/subscribing (new_image)

from cv_bridge import CvBridge # for the bridge between computer vision and ROS 2 image



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

# data from the "k" section above
camera_matrix = [[1061.6538553425996, 0.0, 640.5],[0.0, 1061.6538553425996, 360.5],[0.0, 0.0, 1.0]]

# convert to numpy array
np_camera_matrix = np.array(camera_matrix) 

# distortion coefficients
distortion_co = [0.0, 0.0, 0.0, 0.0, 0.0]

# convert to numpy array
np_distortion_co  = np.array(distortion_co)

# ID of the target ArUco marker
target_ID = 72

# length of marker, at the competition they will be 1 ft by 1 (or 0.3048 m)
marker_length = 0.3048 

# to track the markers that have been detected
detected_markers = []



################### VARIABLES FOR FLIGHT PATH ###################



# connect the drone vehicle to the SITL
print('Connecting to drone...')

# set up the drone vehicle with DroneKit
drone_vehicle = connect("udp:127.0.0.1:14550", wait_ready = True) # connected through UDP

# set land speed to 50 centimeters per second
drone_vehicle.parameters['LAND_SPEED'] = 50 

print('Connected to drone!\n')

global sub_node # global variable for spinning the subscriber node

# tracks how far the drone has travelled. I track this so I know how much to move off the field upon finding the target marker
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
            id = int(returned_ids[0]) # set the variable to the number of the id

            if id == target_ID: # if the ID found is the target ID     
                self.get_logger().info(f'********TARGET MARKER SPOTTED: {id}')  # output the marker was spotted
                
                # once the target marker is found, we want to get off the field and land
                # so we call upon_detection and shutdown the ros2 image subscriber node, as it is not required for landing
                upon_detection()
                rclpy.shutdown()

            else: 
                # I add the id to the list so that the below messaage is not printed to the screen more than once.
                # if it has not already been found, output its id and add it to the list
                if id not in detected_markers: 
                    detected_markers.append(id)
                    self.get_logger().info(f"********Marker detected, but not target marker: {id}")



################### FUNCTIONS SECTION ###################



# function to initiate takeoff
def takeoff():
    # altitude to take off --> I chose 3 meters because I thought it was a safe height to start with
    takeoff_altitude = 3 

    # ensure drone is armable
    print('Waiting for the drone to become armable...')

    while drone_vehicle.is_armable != True:
        time.sleep(1) # sleep until drone is armable

    print("Drone now armable!\n")


    # wait for the drone to be in guided mode
    print('Waiting for the drone to be in GUIDED mode...')
    # set to guided mode
    if drone_vehicle.mode != "GUIDED":
        drone_vehicle.mode = "GUIDED" # set mode to guided
        while drone_vehicle.mode != "GUIDED":
            time.sleep(1) # sleep while mode is not guided

    print("Drone in GUIDED mode!")

    # arm the drone
    if drone_vehicle.armed != True:
        drone_vehicle.armed = True
        print('\nArming the drone...')
        while drone_vehicle.armed != True:
            time.sleep(1)  # sleep while the drone is not armed

        print("Drone now armed, ready for takeoff!")

    # takeoff
    drone_vehicle.simple_takeoff(takeoff_altitude)
    print("Taking off!")

    # while the drone has not reached the target altitude
    while True:
        print(f'\tRising in altitude, currently at {abs(drone_vehicle.location.global_relative_frame.alt):.2f} m') # abs() because sometimes drone outputs negative altitude

        # if the target height is about 95% accurate, break out of the loop
        if drone_vehicle.location.global_relative_frame.alt >= .95 * takeoff_altitude:
            break

        time.sleep(1)

    time.sleep(1)
    print("Target altitude reached, drone takeoff complete!!")
    

# return drone's current lat and lon
def get_current_position():
    return (drone_vehicle.location.global_relative_frame.lat, drone_vehicle.location.global_relative_frame.lon)


# create and send mavlink messaage to ardupilot that holds velocity commands for the drone
def send_velocity(vel_x, vel_y, vel_z):
    global x_travelled
    global y_travelled

    # add to x_travelled and and y_travelled based on the velocity it travels every 0.1 seconds
    # we send this message every .1 second. if drone is flying 0.5 m/s for example, we need to 1/10 of it for our estimation
    x_travelled += vel_x / 10 
    y_travelled += vel_y / 10 

    # create the velocity message
    message = drone_vehicle.message_factory.set_position_target_local_ned_encode(
        0, # time_boot_ms  --> not used
        0, # the target system
        0, # the target componenet
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,# frame, MAV_FRAME_BODY_NED is for no rotation, MAV_FRAME_LOCAL_NED is for rotation
        0b0000111111000111,# type_mask
        0,0,0, # x, y, z positions
        vel_x, # +x --> north, -x --> south
        vel_y, # +y --> east, -x --> west
        vel_z, # +z --> descend, -z --> ascend
        0,0,0, # x, y, z acceleration
        0, # yaw
        0  # yaw_rate
    )

    # send the messaage
    drone_vehicle.send_mavlink(message)
    

# command the drone to fly at a particular velocity until target distance is reached
# stop sending the command when it reaches around the target distance
def control_drone_velocity(start_position, target_distance, vel_x, vel_y, vel_z):

    # get the drone's current location
    current_location = get_current_position()

    # total distance travelled at start of the function
    distance_travelled = 0

    # for only printing out a message every 1 second instead of 0.1
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

        # spin the aruco detection node only while moving
        rclpy.spin_once(sub_node) 

        # send the velocity command every 0.1 seconds
        send_velocity(vel_x, vel_y, vel_z)
        time.sleep(0.1) # sleep for some buffer time

    send_velocity(0,0,0) # stop the drone when target destination is reached


# send drone to specific points within the field
def flight_path():
    print("EXECUTING FLIGHT PATH...")
    time.sleep(1)

    # I used the 'spiral' flight path here, as it was the fastest of the three I creaated (diagonal, spirl, lawnmower 1, and lawnmower 2)
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
    control_drone_velocity(get_current_position(), 7.2, -speed, 0, 0)

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
        while drone_vehicle.location.global_relative_frame.alt > 0.03: # Once on the ground, the altitude of the drone slowly falls from 0.03 to 0
            print("\tLanding..")
            time.sleep(1)

        print("Drone sucessfully landed!")
        
        sys.exit() # exit the script


# upon detecting the aruco marker, move off the field
def upon_detection():
    # retreive the x_travelled
    global x_travelled  

    print("\nAruco Marker was detected. Moving off the field.")

    # go to edge of field
    print(f'Moving off the field.... Flying {(9 - x_travelled):.2f} m forward')
    start_position = get_current_position()
    distance_travelled = 0
    target_distance = 9 - x_travelled # the field's length is around 9 meters, so we retreive how much farther to travel until we reach the end of the field
    
    # send command to vehicle every second (1 hz) until we reach desired distance
    while (distance_travelled < target_distance * 0.95): # threshold so drone stops on time
        current_location = get_current_position()
        distance_travelled = (haversine(start_position, current_location, Unit.KILOMETERS)) * 1000

        send_velocity(0.5, 0, 0) # move 0.5 m/s in the x-axis
        time.sleep(0.1) # send every .1 seconds

    send_velocity(0,0,0) # stop the drone when target destination is reached

    print("Travelled off the field.")

    # land the drone
    land()


# main function --> initilize rclpy for subscriber node, create subscriber node, takeoff, flight path, and land if no aruco marker was found
def main(args = None):
    # set pub sub to global variable. I did this so I could spin the node in other functions
    global sub_node 

    # initalize rclpy (for spinning ROS 2 nodes)
    rclpy.init(args=args)
    
    # create the subscriber node
    sub_node = ImageSub() 

    # function to takeoff
    takeoff()
    time.sleep(1) # buffer time

    # start the flight path
    flight_path()

    # if the full flight path was completed, then the target marker was not detected
    land() # land the drone

    print("Target Aruco marker not detected.")


# initiate main
if __name__ == '__main__': 
    main()