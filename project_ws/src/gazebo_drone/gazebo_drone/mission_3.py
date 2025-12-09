#!/usr/bin/env python3 

"""

Challenge 3 Flight Path/Ground Vehicle Logic

Challenge 3:
    Upon drone takeoff, with the drone reaching a minimum altitude of 4 feet, the ground vehicle begins moving at at least .2 mph (~0.087 m/s) in a straight line.
    After waiting 5 seconds, the drone will move forward.

    Upon detecting the ground vehicle, the drone will perform a precision landing.

    
"""

import rclpy # ros2 package

import numpy as np # for turing the image array into numpy arrays

import cv2 # for detecting aruco markers using computer vision

import sys # for exiting the script

import time # for helping with creating realistic frame rate

from cv_bridge import CvBridge # for the bridge between computer vision and ros2

from dronekit import connect # the dronekit API assists with communicating with the ArduPilot flight controller firmware

from pymavlink import mavutil # the paymavlink API assists with creating custom MAVLink messages to send to ArduPIlot

from sensor_msgs.msg import Image # message type for our topic we are publishing/subscribing (new_image)

from rclpy.node import Node # for creating publisher and subscriber nodes

import threading # for spinning the ImageSub node and running the main script

from cart_publisher import CartNode # publishes velocity for the cart to travel at

from cart_subscriber import CartSubNode # retreives cart's current velocity

from rclpy.executors import MultiThreadedExecutor # using threading for spinning multiple nodes

import socket # for sending/recieving messages



################### VARIABLES SECTION ###################



# set up the vehicle with drone kit
print('Connecting to drone...')
drone_vehicle = connect("tcp:127.0.0.1:5763", wait_ready = True)
print('Connected.')

# target aruco marker ID
target_ID = 72  

# holds whether target marker was found
detected = False



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

threshold = 0 

marker_length = 0.15 # 0.15 m

# for estimating camera distance from target marker 
aruco_points = np.array([
    [-marker_length/2, marker_length/2, 0], 
    [marker_length/2, marker_length/2, 0],
    [marker_length/2, -marker_length/2, 0],
    [-marker_length/2, -marker_length/2, 0],
    ]
    )

# create a UDP socket for receiving message from cart
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # internet, udp
sock.bind(('127.0.0.1', 14650))
sock.settimeout(0.01) # maximum time to wait for receiving data



################### SUBSCRIBER SECTION ###################



# subscriber to retreive the camera feed (type, topic, queue size)
class ImageSub(Node):
    def __init__(self):
         # create the subscriber
        super().__init__('image_subscriber')
        self.subscriber = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)

    def callback(self, message):
        global detected

        bridge = CvBridge()
        
        # convert ROS Image message to OpenCV iamge
        np_data = bridge.imgmsg_to_cv2(message, desired_encoding='rgb8') # convert ROS Image message to openCV Image (which is a numpy array)
        
        # get image and convert to grayscale for better accuracy
        gray_image = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)
    
        # detect aruco markers in frame
        # get the corners of the aruco markers and the IDs that it returned with detectMarkers function
        (corners, returned_ids, rejected) = aruco_detector.detectMarkers(gray_image)

        # try to retrieve message from cart
        try:
            data, address = sock.recvfrom(1024) # buffer size of 1024

            data_decoded = data.decode('utf-8')
            data_decoded = data_decoded.split(", ")
            # absolute values -- we assign direction later in code
            cart_x_velocity  = abs(float(data_decoded[0]))
            cart_y_velocity = abs(float(data_decoded[1]))
            print(f"Received velocity message from cart: \n\t{cart_x_velocity} m/s in x-axis, \n\t{cart_y_velocity} m/s in y-axis")
        
        # if failed, output didn't receive message
        # sets velocity of drone to a safe velocity option (0.1 m/s)
        except socket.timeout:
            #print("Didn't recieve velocity message from cart. Setting x_velocity to .1 m/s.")
            cart_x_velocity = 0.1
            cart_y_velocity = 0.0 # the y velocity should be zero for challenge 1

        # threshold for distance between marker and center of camera
        threshold = 0.10

        # velocity for z axis to descend
        descending = 0.10

        # velocitys to move faster than the cart
        faster_x = cart_x_velocity + 0.1
        faster_y = cart_y_velocity + 0.1

        # if an ID is found
        if returned_ids is not None: 

            # if the ID found is the target ID
            if returned_ids[0] == target_ID: 
                detected = True
                
                self.get_logger().info(f'*****LAND ZONE IN SIGHT')

                # get the x and y distances from aruco's center
                returned, rvec, tvec = cv2.solvePnP(aruco_points, corners[0], np_camera_matrix, np_distortion_co)

                # calculate Xerror/distance between camera and aruco (in centimeters)
                X = round(tvec[0][0], 2) # left/right distance
                Y = round(tvec[1][0], 2) # up/down distance

                #print(f"left/right distance from aruco, {X} m")
                #print(f"up/down distance from aruco {Y} m")

                print('\nALTITUDE', drone_vehicle.location.global_relative_frame.alt)

                # if left/right and up/down are greater than threshold
                # move both about .1 faster than the received velocity
                if abs(Y) > threshold and abs(X) > threshold:
                    #print("moving... diagonal")
                    # if drone is ahead, slow down
                    send_velocity(faster_x if Y < 0 else 0.05, -faster_y if X < 0 else faster_y, descending)
                
                # if left/right is greater and up/down is not,
                # move faster than cart in x axis but same in y axis
                elif abs(Y) > threshold and abs(X) < threshold:
                    #print("moving... up/down")
                    # if drone is ahead, slow down
                    send_velocity(faster_x if Y < 0 else 0.05, cart_y_velocity, 0.1)
                
                # if left/right is less than and up/down is greater,
                # move faster than cart in y axis but same in x axis
                elif abs(X) > threshold and abs(Y) < threshold:
                    #print("moving... left/right")
                    send_velocity(cart_x_velocity, -faster_y if X < 0 else faster_y, descending)

                # otherwise, move at the x and y axis velocities and descend.
                else:
                    #print("directly overhead aruco!")
                    send_velocity(cart_x_velocity, cart_y_velocity, descending)

                time.sleep(0.3) # some buffer time so its not too shakey
        
        # if a marker was not found
        else:
            # if the marker was detected and its altitude is less than 0.8 m and the altitude is greater than 0.4 m
            # then keep moving forward with the cart and descend a bit faster
            if detected:
                if drone_vehicle.location.global_relative_frame.alt < 0.8 and drone_vehicle.location.global_relative_frame.alt > 0.4:
                    print('LANDING...')
                    send_velocity(cart_x_velocity + 0.08, cart_y_velocity, 0.2)
                
                # if the altitude is less than 0.4, then the drone landed on the platform
                elif drone_vehicle.location.global_relative_frame.alt < 0.4:
                    landed_on_platform()



################### FUNCTIONS SECTION ###################



# send a message that forcibly disarms the drone
# we need to do this because the drone is not acknowledging that it has landed despite it's altitude reflecting otherwise
# this is because the drone is still moving on the cart, and so ArduPilot perceives the drone as still moving
def force_disarm_vehicle():

    # create and send message to force disarm the vehicle
    drone_vehicle.message_factory.command_long_send(
        0, # target system
        0, # target_component
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, # command (also equal to 400)
        0, # confirmation
        0, # param 1 (0:disarm, 1:arm)
        21196, # param 2 (0:disarm unless prevented by safety checks, 21196:force arming or disarming
        0, # not used -- param 3 - 7 below are not used
        0, # not used
        0, # not used
        0, # not used
        0, # not used
    )


# called when the drone lands on the platform
def landed_on_platform():
    global executor
    global challenge_thread

    print("\n*******************LANDED ON PLATFORM*******************")
    # stop moving in all axises
    # send this message multiple times to ensure the drone stops
    for i in range (3):
        send_velocity(0,0,0)

    # force disarm the drone
    print("Disarming the drone.")
    while drone_vehicle.armed:
        force_disarm_vehicle()
        time.sleep(0.5)

    print("Drone disarmed.")
    drone_vehicle.close() # disconnect vehicle from ArduPilot

    # shut down all nodes
    executor.shutdown()
    rclpy.shutdown()

    sys.exit()
    print("System exit.")


# function to initiate takeoff
def takeoff():
    # altitude to take off
    # must takeoff at a minimum of 4 feet, which is about 1.2 meters.
    takeoff_altitude = 3 

    # ensure drone is armable
    print('\nWaiting for the drone to become armable...')

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
    print("Taking off!")

    # output altitude while taking off
    while True:
        print(f'\tRising in altitude, currently at {abs(drone_vehicle.location.global_relative_frame.alt):.2f} m') # abs() because sometimes drone outputs negative altitude

        # if the target height is about 95% accurate, break out of the loop
        if drone_vehicle.location.global_relative_frame.alt >= .95 * takeoff_altitude:
            break

        time.sleep(1)

    time.sleep(1)
    print("Target altitude reached, drone takeoff complete!!")


# send a velocity command in each axis with MAVLink message 
def send_velocity(vel_x, vel_y, vel_z):

    # create the message
    message = drone_vehicle.message_factory.set_position_target_local_ned_encode(
        0,# time_boot_ms -- we don't use this
        0,# target system
        0,# target componenet
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,# frame, MAV_FRAME_LOCAL_NED is for no rotation, MAV_FRAME_BODY_NED is for rotation
        0b0000111111000111,# type_mask
        0,0,0,# x, y, z positions
        vel_x, # +x --> north, -x --> south
        vel_y, # +y --> east, -x --> west
        vel_z, # +z --> descend, -z --> ascend
        0,0,0,# x, y, z acceleration
        0,# yaw
        0 # yaw_rate
    )

    # send the message
    drone_vehicle.send_mavlink(message)


# fly forward until aruco marker is found
def flight_path():
    time.sleep(1)

    # wait 5 seconds before moving forward, as per challenge requirements
    for i in range(1, 6):
        print(f"Moving forward in {i}...")
        time.sleep(1)
    
    print("Moving forward!")

    # while the aruco marker is not in sight, slowly move forward
    while True:
        global detected
        if not detected:
            send_velocity(0.5, 0, 0)


# main function
def main():
    # initiate rclpy
    rclpy.init(args=None)

    # create executor for spinning multiple nodes
    global challenge_thread
    challenge_thread = threading.Thread(target=flight_path)
    challenge_thread.daemon = True # this allows the thread to stop even when the file stops

    # cart node, aruco node, odom node
    cart_node = CartNode()
    aruco_node = ImageSub()
    cart_sub_node = CartSubNode()

    # for spinning multiple nodes at once
    global executor
    executor = rclpy.executors.MultiThreadedExecutor()

    # add nodes to the executor
    executor.add_node(cart_node)
    executor.add_node(aruco_node)
    executor.add_node(cart_sub_node)

    # takeoff
    takeoff()

    print("\nTakeoff Complete. Sending instruction to cart, then waiting 5 seconds...")

    # start thread that will have drone move forward
    challenge_thread.start()

    # spin the executor unless the user cancels it
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    # destroy the nodes
    cart_node.destroy_node()
    aruco_node.destroy_node()
    cart_sub_node.destroy_node()

    # join the thread
    challenge_thread.join()
        

# initiate main
if __name__ == '__main__': 
    main()

