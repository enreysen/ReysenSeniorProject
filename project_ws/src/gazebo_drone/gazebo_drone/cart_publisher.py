"""


For Challenge 3 Flight Path Logic

Publishes velocity for ground vehicle to travel. Must travel at a minimum of 0.2 mph, or 0.089408 m/s

  
"""

# messages must be in geometry_msgs/msg/Twist
# topic name: cmd_vel

from rclpy.node import Node
from geometry_msgs.msg import Twist

#0.089408  # velocity in x-direction (linear) 0.089408 is around .2 mph

class CartNode(Node):
    def __init__(self):
        super().__init__('cart_publisher')
        self.publisher = self.create_publisher(Twist, 'cart/cmd_vel', 10)

        timer_period = 0.05 # 1 / 30  seconds is how often the camera publishes.
        self.timer = self.create_timer(timer_period, self.timer_callback)


# need to achieve at least velocity of 0.089408 m/s

    def timer_callback(self):
            # check if it has recieved a message. if so, set the velocities to those numbers

            # if not, publish the regular velocity of 1.2
            velocity_msg = Twist() # messaage for cart (takes message of type Twist)

            velocity_msg.linear.x = -1.2
            velocity_msg.linear.y = 0.0 
            velocity_msg.linear.z = 0.0

            # publish the message
            self.publisher.publish(velocity_msg)


# uncomment below if wanting to test the node without running mission_3.py
# def main():
#     rclpy.init()
#     cart = CartNode()

#     print("Sending velocity...")
#     rclpy.spin(cart)
#     rclpy.shutdown()


# main()