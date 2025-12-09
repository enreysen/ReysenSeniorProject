#!/usr/bin/env python3 

"""

Challenge 2 Flight Path Logic (UAV 2)

Challenge 2 UAV 2:
    Receive travel instructions from UAV 1
    Fly to aruco marker location
    Preform a precision landing upon detecting the marker

    
"""

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

import socket # for sending/recieving messages

import threading # for spinning the ImageSub node and running the main script

import math # using pi for vertical fov calculation


################### VARIABLES FOR ARUCO DETECTION ###################

# specify aruco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL) 

# specify aruco parameters
aruco_parameters = cv2.aruco.DetectorParameters()

# aruco detector
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_parameters)

# holds marker IDs that were detected
detected_markers = []

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

# from the "k" section above
camera_matrix = [[1061.6538553425996, 0.0, 640.5],[0.0, 1061.6538553425996, 360.5],[0.0, 0.0, 1.0]]

# convert to numpy array
np_camera_matrix = np.array(camera_matrix) 

distortion_co = [0.0, 0.0, 0.0, 0.0, 0.0]

# convert to numpy array
np_distortion_co  = np.array(distortion_co) 

# width in camera sdf file
horizontal_res = 1280 

# height in camera sdf file
vertical_res = 720 

# from camera sdf file
horizontal_fov = 1.085 
vertical_fov = 48.8 * (math.pi / 180)

# target aruco marker ID
target_ID = 72 

# at competition, marker length with be 1 ft x 1 ft  (0.3048 m)
marker_length = 0.3048

# used in logic later on in script
detected = False

# for detecting distance from camera to marker, important for precision landing
aruco_points = np.array([
    [-marker_length/2, marker_length/2, 0], 
    [marker_length/2, marker_length/2, 0],
    [marker_length/2, -marker_length/2, 0],
    [-marker_length/2, -marker_length/2, 0],
    ]
    )

# connect the secondary vehicle to the SITL
print('Connecting to secondary vehicle...') # with I1 tag


# set up the vehicle with drone kit
secondary_vehicle = connect("udp:127.0.0.1:14560", wait_ready = True) # was tcp:127.0.0.1:5763

# precision landing enabled
secondary_vehicle.parameters['PLND_ENABLED'] = 1

# target position source from MAVLINK
secondary_vehicle.parameters['PLND_TYPE'] = 1 

# raw sensor, not kalman filter
secondary_vehicle.parameters['PLND_EST_TYPE'] = 0 

# land speed to 50 centimeters per second
secondary_vehicle.parameters['LAND_SPEED'] = 50

print('Connected to secondary vehicle!\n')



################### SUB NODE SECTION ###################




# publisher to publish the camera feed (type, topic, queue size)
class ImageSub(Node):
    def __init__(self):
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
            id = int(returned_ids[0]) # if ID has not been found yet

            if returned_ids[0] == target_ID: # if the ID found is the target ID     

                if id not in detected_markers: # add target ID to list
                        detected_markers.append(id) # for only printing target marker was detected once
                        self.get_logger().info(f"\n********TARGET ARUCO DETECTED: {id}")
                        print("Starting precision landing process.")

                # set detected to True so that the drone can start precision landing
                global detected 
                detected = True

                # get the x and y distances from aruco's center
                returned, rvec, tvec = cv2.solvePnP(aruco_points, corners[0], np_camera_matrix, np_distortion_co)

                # calculate Xerror/distance between camera and aruco (in centimeters)
                X = round(tvec[0][0], 2) # left/right distance
                Y = round(tvec[1][0], 2) # up/down distance

                # threshold for acceptable distance from target marker
                threshold = 0.10 

                # speed for descending
                descending = 0.10 

                # velocity values
                velocity_x = 0.1 if Y < 0 else -0.1
                velocity_y = -0.1 if X < 0 else 0.1

                # if the drone's distance is greater than the threshold in the left/right and up/down direction
                # move diagonally toward aruco marker
                if abs(Y) > threshold and abs(X) > threshold:
                    print("Moving... diagonal")
                    send_velocity(velocity_x, velocity_y, descending)
                
                # if the drone's distance is greater than the threshold in the up/down direction
                # move up/down toward aruco marker
                elif abs(Y) > threshold and abs(X) < threshold:
                    print("Moving... up/down")
                    send_velocity(velocity_x, 0, descending)
                
                # if the drone's distance is greater than the threshold in the left/right
                # move left/right toward aruco marker
                elif abs(X) > threshold and abs(Y) < threshold:
                    print("Moving... left/right")
                    send_velocity(0, velocity_y, descending)

                # if the drone's distance is below the threshold for both x and y axises
                # simply descend
                else:
                    send_velocity(0, 0, descending)

                # buffer time so drone doesn't move shakily
                time.sleep(0.5)

        # if the aruco marker is not detected currently but has been detected in the past, and its altitude is less than 0.04, then the drone has landed   
        # at a certain height, the marker is not visible anymore even though the drone is hovering above the marker
        else:
            if detected and secondary_vehicle.location.global_relative_frame.alt <= 0.04:
                print("Secondary vehicle landed.")

                # exit the script
                sys.exit()

                # shut down rclpy
                rclpy.shutdown()
            
            # if the marker has been detected but is not visible now and its altitude is greater than 0.04, then keep descending
            elif detected and secondary_vehicle.location.global_relative_frame.alt >= 0.04:
                send_velocity(0, 0, 0.1)
                



################### FUNCTIONS SECTION ###################



# secondary drone receive message over serial port
def receive_message():

    # Create a UDP socket
    sock = socket.socket(socket.AF_INET,  # internet
                        socket.SOCK_DGRAM) # udp

    # receive the message
    sock.bind(('127.0.0.1', 14650))

    while True:
        data, address = sock.recvfrom(1024) # buffer size of 1024

        data_decoded = data.decode('utf-8')
        data_decoded = data_decoded.split(", ")

        # retreive the x and y distances
        x = float(data_decoded[0])
        y = float(data_decoded[1])

        print(f"Received message from primary: {x:.2f} {y:.2f}")

        # return x and y distances
        return x, y
    

# function to initiate takeoff
def takeoff():
    # altitude to takeoff
    takeoff_altitude = 3.5

    # ensure drone is armable
    print('Waiting for the drone to become armable...')
    while secondary_vehicle.is_armable != True:
        time.sleep(1)

    print("Secondary now armable!\n")

    # set to guided mode
    if secondary_vehicle.mode != "GUIDED":
        secondary_vehicle.mode = "GUIDED"
        print('Waiting for the drone to be in GUIDED mode...')
        while secondary_vehicle.mode != "GUIDED":
            time.sleep(1)

        print("Secondary now in GUIDED mode!")

     # arm the drone
    if secondary_vehicle.armed != True:
        secondary_vehicle.armed = True
        print('Arming the drone...')
        while secondary_vehicle.armed != True:
            time.sleep(1)

        print("Secondary now armed, ready for takeoff!")

    # takeoff
    secondary_vehicle.simple_takeoff(takeoff_altitude)

    print("Taking off!")

    # print out current altitude when taking off 
    while True:
        time.sleep(1)
        print(f'\tRising in altitude, currently at {secondary_vehicle.location.global_relative_frame.alt:.2f} m')

        # if the target height is about 95% accurate
        if secondary_vehicle.location.global_relative_frame.alt >= .95 * takeoff_altitude:
            break

    time.sleep(1)
    print("Target altitude reached, secondary vehicle takeoff complete!")


# return drone's current lat and lon
def get_current_position():
    return (secondary_vehicle.location.global_relative_frame.lat, secondary_vehicle.location.global_relative_frame.lon)


# send  message with velocity commands to ArduPilot with Pymavlink and DroneKIt
def send_velocity(vel_x, vel_y, vel_z):
    message = secondary_vehicle.message_factory.set_position_target_local_ned_encode(
        0,# time_boot_ms -- we don't use this
        0,# target system
        0,# target componenet
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,# frame, MAV_FRAME_BODY_NED is for no rotation, MAV_FRAME_LOCAL_NED is for rotation
        0b0000111111000111,# type_mask
        0,0,0, # x, y, z positions
        vel_x, # +x --> north, -x --> south
        vel_y, # +y --> east, -x --> west
        vel_z, # +z --> descend, -z --> ascend
        0,0,0, # x, y, z acceleration
        0, # yaw
        0 # yaw_rate
    )

    # send the message to flight controller firmwaare
    secondary_vehicle.send_mavlink(message)
    

# command the drone to fly at a particular velocity until target distance is reached
def control_drone_velocity(start_position, target_distance, vel_x, vel_y, vel_z):
    distance_travelled = 0
    
    # send command to vehicle every second (1 hz) until we reach desired distance
    while (distance_travelled < target_distance * 0.95 and detected == False): # threshold so drone stops on time
        current_location = get_current_position()
        distance_travelled = (haversine(start_position, current_location, Unit.KILOMETERS)) * 1000
        
        #print(f"\tDistance from target: {target_distance - distance_travelled}")
        
        # call function that will create the velocity message
        send_velocity(vel_x, vel_y, vel_z)

        # buffer time
        time.sleep(0.1) 
        

    # stop the drone when target destination is reached
    send_velocity(0,0,0) 


# function for moving toward the aruco based on distance estimates from primary drone
def move_to_aruco(x, y):
    # move forward to aruco
    print("\nMoving forward to aruco")

    # move forward initially to where the primary drone started
    control_drone_velocity(get_current_position(), 1, 0.5, 0, 0)

    # continue moving forward to aruco based on distance shared from primary
    velocity_x = -0.50 if x < 0 else 0.50 # if the x distance is below zero, set the velocity to be negative
    control_drone_velocity(get_current_position(), round(abs(x),2), velocity_x, 0, 0)

    print("Moving right/left to aruco\n")
    # move left/right to aruco
    velocity_y = -0.50 if y < 0 else 0.50
    control_drone_velocity(get_current_position(), round(abs(y) + 1,2), 0, velocity_y, 0)


# function for spinning node required for precision landing
def precision_land():
    # start rclpy for spinning nodes
    rclpy.init(args=None)

    # create node
    image_sub = ImageSub()

    # spin node
    rclpy.spin(image_sub)


# main function
def main():
    print('Waiting on message from primary drone...')
    # receive message, then takeoff
    distances = receive_message()    

    # distance to travel in x and y axis
    x_to_travel = distances[0]
    y_to_travel = distances[1]

    # takeoff the drone
    takeoff()

    # threads for spinning node and moving to the aruco marker logic
    thread1 = threading.Thread(target=precision_land)
    thread2 = threading.Thread(target=move_to_aruco, args=(x_to_travel, y_to_travel))

    # start nodes
    thread1.start()
    thread2.start()


# initiate main
if __name__ == '__main__': 
    main()