#!/usr/bin/env python3 

"""


Challenge 2 Flight Path Logic (UAV 1)

Challenge 2 UAV 1:
    Autonomously search the field to find the target aruco marker
    Upon finding the target marker, output that it was found, and send aruco marker location information to UAV 2
    Then, fly off the field and land.

  
"""

# aruco tracking and primary vehicle flight path in one
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

import socket # for sending/recieving messages



################### VARIABLES FOR ARUCO DETECTION ###################



# specify aruco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL) 

# specify aruco parameters
aruco_parameters = cv2.aruco.DetectorParameters()

# aruco detector
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_parameters)

# dictionary for holding the detected markers.
detected_markers = []

# camera calibration (camera matrix and distortion coefficients)
# find by running the gazebo world and viewing ros2 topic list /camera/camera_image

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


# points of the marker for retreiving rotational and translation vectors from center of aruco
aruco_points = np.array([
    [-marker_length/2, marker_length/2, 0], 
    [marker_length/2, marker_length/2, 0],
    [marker_length/2, -marker_length/2, 0],
    [-marker_length/2, -marker_length/2, 0],
    ]
    )



################### IMAGE SUBSCRIBER SECTION ###################



# subscriber to retreive image data from drone camera
class ImageSub(Node):
    def __init__(self):
        # initialize the node
        super().__init__('image_sub_1')

        # create the subscriber
        self.subscriber = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.subscriber


    def callback(self, message):
        #self.get_logger().info(f'Retrieving camera/image_raw data')
        bridge = CvBridge() # for bridging between ROS 2 and OpenCV image
        
        # convert ROS Image message to OpenCV iamge
        np_data = bridge.imgmsg_to_cv2(message, desired_encoding='rgb8') # convert ROS Image message to openCV Image (which is a numpy array)
        
        # get image and convert to grayscale for better accuracy
        gray_image = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)
    
        # detect aruco markers in frame
        # get the corners of the aruco markers and the IDs that it returned with detectMarkers function
        (corners, returned_ids, rejected) = aruco_detector.detectMarkers(gray_image)

        # if an ID is found
        if returned_ids is not None: 
            # set id variable to the id number
            id = int(returned_ids[0])

            # if the ID found is the target ID     
            if id == target_ID: # get the distance of the drone from the marker
                
                # add the id to the detected markers list
                if id not in detected_markers:
                    detected_markers.append(id)

                    # print marker was found (get_logger returns timestamp as well)
                    self.get_logger().info(f'********TARGET MARKER SPOTTED: {id}')

                # get the x and y distances from aruco's center
                returned, rvec, tvec = cv2.solvePnP(aruco_points, corners[0], np_camera_matrix, np_distortion_co)

                x_distance_center = tvec[0] # left/right distance from marker
                y_distance_center = tvec[1] # up/down distance from marker

                self.get_logger().info(f'********DRONE DISTANCE FROM CENTER OF ARUCO: LEFT/RIGHT:{x_distance_center[0]:.2f} UP/DOWN:{y_distance_center[0]:.2f}')
                
                # return to launch/precision land
                # y is up/down and x is left/right
                upon_detection(y_distance_center[0], x_distance_center[0])
                rclpy.shutdown()


            # if the if an id has been found but is not in the detected markers list
            else: # output the marker has been found
                if id not in detected_markers:
                    detected_markers.append(id)
                    self.get_logger().info(f"********Marker detected, but not target marker: {id}")



# connect the primary vehicle to the SITL
print('Connecting to primary vehicle...') # with I1 tag

# set up the vehicle with drone kit
primary_vehicle = connect("udp:127.0.0.1:14550", wait_ready = True)
primary_vehicle.parameters['LAND_SPEED'] = 50 # land speed

print('Connected to primary vehicle!\n')

# total distance primary drone has travelled in x and y axises
x_travelled = 0
y_travelled = 0

# sub node global as we need to spin it in other functions
global sub_node

# holds whether marker has been found
detected = False



################### FUNCTIONS SECTION ###################



# send message from primary drone over serial port (x_distance travelled and y_distance travelled)
def send_message(x_travel, y_travel):

    # create the message
    message = f"{x_travel}, {y_travel}"
    encoded_message = message.encode('utf-8')
    
    # print(f"Message: {message}")

    # create a UDP socket
    sock = socket.socket(socket.AF_INET,  # internet
                        socket.SOCK_DGRAM) # udp

    # send the message
    sock.sendto(encoded_message, ('127.0.0.1', 14650))

    return


# function to initiate takeoff
def takeoff():

    # altitude for drone to fly at
    takeoff_altitude = 3 

    # ensure drone is armable
    print('Waiting for the drone to become armable...')

    while primary_vehicle.is_armable != True:
        time.sleep(1)

    print("Primary now armable!\n")

    # set the drone to GUIDED mode
    print('Waiting for the drone to be in GUIDED mode...')
    if primary_vehicle.mode != "GUIDED":
        primary_vehicle.mode = "GUIDED"
        while primary_vehicle.mode != "GUIDED":
            
            time.sleep(1)

        print("Primary now in GUIDED mode!")

    # arm the drone
    if primary_vehicle.armed != True:
        primary_vehicle.armed = True
        print('Arming the drone...')
        while primary_vehicle.armed != True:
            time.sleep(1)

        print("Primary now armed, ready for takeoff!")

    # takeoff
    primary_vehicle.simple_takeoff(takeoff_altitude)
    print("Taking off!")

    # output altitude
    while True:
        time.sleep(1)
        print(f'\tRising in altitude, currently at {primary_vehicle.location.global_relative_frame.alt:.2f} m')

        # if the target height is about 95% accurate
        if primary_vehicle.location.global_relative_frame.alt >= .95 * takeoff_altitude:
            break

    time.sleep(1)
    print("Target altitude reached, primary drone takeoff complete!!")
    

# function for creating and sending velocity commands to ArduPilot
def send_velocity(vel_x, vel_y, vel_z):
    global x_travelled
    global y_travelled

    # we send this message every .1 second. if drone is flying 0.5 m/s for example, we need to 1/10 of it for our estimation
    x_travelled += vel_x / 10 
    y_travelled += vel_y / 10 

    # create message
    message = primary_vehicle.message_factory.set_position_target_local_ned_encode(
        0, # time_boot_ms -- we don't use this
        0, # target system
        0, # target componenet
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

    # send message
    primary_vehicle.send_mavlink(message)
    

# command the drone to fly at a particular velocity until target distance is reached
def control_drone_velocity(start_position, target_distance, vel_x, vel_y, vel_z):
    distance_travelled = 0
    print_time = time.time()
    
    # send command to vehicle every second (1 hz) until we reach desired distance
    while (distance_travelled < target_distance * 0.95): # threshold so drone stops on time
        current_location = get_current_position()
        distance_travelled = (haversine(start_position, current_location, Unit.KILOMETERS)) * 1000


        if (time.time() - print_time > 1):
            #print(f"\tDistance from target: {target_distance - distance_travelled}")
            print_time = 0


        # spin node only once
        rclpy.spin_once(sub_node) 
        
        # call function that creates and sends velocity message
        send_velocity(vel_x, vel_y, vel_z)

        # buffer time
        time.sleep(0.1) 

    # stop the drone when target destination is reached
    send_velocity(0,0,0) 


# return drone's current lat and lon
def get_current_position():
    return (primary_vehicle.location.global_relative_frame.lat, primary_vehicle.location.global_relative_frame.lon)


# send drone to specific points within the field
def flight_path():
    print("INITIATING FLIGHT PATH")
    time.sleep(1)

    # lawn mower flight path

    # move up ~1 m
    control_drone_velocity(get_current_position(), 0.30, 0.5, 0, 0)

    for i in range(2):
        # move to the right ~7 meters
        control_drone_velocity(get_current_position(), 6.5, 0, 0.5, 0)

        # move up ~2 m
        control_drone_velocity(get_current_position(), 1.25, 0.5, 0, 0)

        # move to the left ~7 meters
        control_drone_velocity(get_current_position(), 6.5, 0, -0.5, 0)

        # move up ~2 m
        control_drone_velocity(get_current_position(), 1.25, 0.5, 0, 0)


    # move to the right ~7 meters
    control_drone_velocity(get_current_position(), 6.5, 0, 0.5, 0)

    # move up ~1 m
    control_drone_velocity(get_current_position(), 0.75, 0.5, 0, 0)
    

# function to land the drone
def land():
    print("INITIATING LANDING")
    # set to land mode
    if primary_vehicle.mode != "LAND":
        primary_vehicle.mode = "LAND"
        while primary_vehicle.mode != "LAND":
            print('Waiting for the drone to be in LAND mode...')
            time.sleep(1)

        print("Initating landing!")
        while primary_vehicle.location.global_relative_frame.alt > 0.07:
            print("\tLanding..")
            time.sleep(1)

        print("Primary drone sucessfully landed!")

        # exit the script after landing
        sys.exit()


# pass in the x distance from the center of the aruco and the y distance (as x_distance) from center of arcuo
def upon_detection(add_x_distance, add_y_distance):
    global x_travelled
    global y_travelled

    print("\nAruco Marker was detected. Initiating landing procedure.")

    # combine the distances for a more accurate distance for the drone to travel
    print(f"***X_travelled: {x_travelled:.2f}")
    print(f"***Y_travelled: {y_travelled:.2f}")
    total_x = -add_x_distance + x_travelled
    total_y = add_y_distance + y_travelled

    print(f"Sending secondary drone to travel {total_x:.2f} up and {total_y:.2f} left/right")

    # send message to primary drone
    send_message(total_x, total_y) # send message after moving forward
    time.sleep(1)
    send_message(total_x, total_y) # send message after moving forward
    time.sleep(1)
    send_message(total_x, total_y) # send message after moving forward

    # go to edge of field
    print(f'Moving off the field.... Flying {(9 - x_travelled):.2f} m forward')
    start_position = get_current_position(); # start position lat and lon

    # current distance travelled
    distance_travelled = 0

    # distance to travel off the field
    target_distance = 9 - x_travelled
    
    # send command to vehicle every second (1 hz) until we reach desired distance
    while (distance_travelled < target_distance * 0.95): # threshold so drone stops on time
        current_location = get_current_position()
        distance_travelled = (haversine(start_position, current_location, Unit.KILOMETERS)) * 1000

        send_velocity(0.5, 0, 0)
        time.sleep(0.1) # buffer time


    # stop the drone when target destination is reached
    send_velocity(0,0,0) 

    print("Travelled off the field.")

    # land the drone
    land()

def main(args = None):
    global sub_node
    # initalize rclpy for spinning node
    rclpy.init(args=args)
    sub_node = ImageSub() # create the pubsub

    # take off the drone
    takeoff()
    time.sleep(1)

    # initiate flight path for drone
    flight_path()

    # if the drone finishes the flight path, then the target marker was not found
    land()

    print("Primary finished flight path. Aruco marker not detected")


# initiate main
if __name__ == '__main__': 
    main()