# messages must be in geometry_msgs/msg/Twist
# topic name: cmd_vel

import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import socket

#0.089408  # velocity in x-direction (linear) 0.089408 is around .2 mph

"""
    Example output from /cart/odom

       geometry_msgs.msg.TwistWithCovariance(twist=geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=0.0033932046243131736, y=0.002652566341048414, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=-0.000914521310620735)), covariance=array([1.e-05, 0.e+00, 0.e+00, 0.e+00, 0.e+00, 0.e+00, 0.e+00, 1.e-05,
       0.e+00, 0.e+00, 0.e+00, 0.e+00, 0.e+00, 0.e+00, 1.e+12, 0.e+00,
       0.e+00, 0.e+00, 0.e+00, 0.e+00, 0.e+00, 1.e+12, 0.e+00, 0.e+00,
       0.e+00, 0.e+00, 0.e+00, 0.e+00, 1.e+12, 0.e+00, 0.e+00, 0.e+00,
       0.e+00, 0.e+00, 0.e+00, 1.e-03]))

"""

class CartSubNode(Node):
    def __init__(self):
        super().__init__('cart_odom_subscriber')
        self.subscriber = self.create_subscription(Odometry, '/cart/odom', self.callback, 10)


# need to achieve at least velocity of 0.089408 m/s
    def callback(self, message):
        velocity_msg_x = message.twist.twist.linear.x
        velocity_msg_y = message.twist.twist.linear.y
        velocity_msg_z = message.twist.twist.linear.z

        total_velocity = math.sqrt(velocity_msg_x**2 + velocity_msg_y**2)

        #print(f"Cart's total velocity: {total_velocity:.2f} m\s")

        # send velocities as a message for the drone to use
        # create the message
        msg = f"{velocity_msg_x:.2f}, {velocity_msg_y:.2f}, {velocity_msg_z:.2f}"
        encoded_message = msg.encode('utf-8')
        
        # create a UDP socket
        sock = socket.socket(socket.AF_INET,  # internet
                            socket.SOCK_DGRAM) # udp

        # send the message
        sock.sendto(encoded_message, ('127.0.0.1', 14650))
        time.sleep(0.3) # some buffer time


# uncomment below if you want to test the node without running mission_3.py
# def main():
#     rclpy.init()
#     cart = CartSubNode()

#     print("getting...")
#     rclpy.spin(cart)
#     rclpy.shutdown()


# main()