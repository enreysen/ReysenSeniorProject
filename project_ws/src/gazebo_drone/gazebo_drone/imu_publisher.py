import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from gazebo_drone.original import get_vehicle


class ImuPublisher(Node):

    def __init__(self, drone_vehicle):
        super().__init__('imu_publisher')
        self.publisher = self.create_publisher(Imu, 'imu', 10)
        self.drone_vehicle = drone_vehicle

        @self.drone_vehicle.on_message('RAW_IMU')
        def get_imu(self2, name, message):
            imu_msg = Imu() # we need to publish message from IMU with the linear acceleration values and angular velocity values for the ORBSLAM3 algorithm

            # get the linear acceleration on each axis
            imu_msg.linear_acceleration.x = float(message.xacc)
            imu_msg.linear_acceleration.y = float(message.yacc)
            imu_msg.linear_acceleration.z = float(message.zacc)

            # get the angular velocity on each axis
            imu_msg.angular_velocity.x = float(message.xgyro)
            imu_msg.angular_velocity.y = float(message.ygyro)
            imu_msg.angular_velocity.z = float(message.zgyro)

            self.publisher.publish(imu_msg) # publish message to 'imu' topic
            self.get_logger().info("Published IMU message") #output that message was published


def main(args=None):
    rclpy.init(args=args)
    vehicle = get_vehicle()
    print("Vehicle connected:", vehicle)

    imu_publisher = ImuPublisher(vehicle)
    rclpy.spin(imu_publisher)

    imu_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
