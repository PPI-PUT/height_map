import rclpy
from rclpy.node import Node
from rclpy import qos
from geometry_msgs.msg import PointStamped, Pose, PoseStamped

import numpy as np


class PosePub(Node):
    def __init__(self):
        super().__init__("pose_pub")
        self.get_logger().info("Pose Pub Node has been started")

        self.pose_pub = self.create_publisher(
            PoseStamped, "/pose", qos.qos_profile_sensor_data
        )

        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = PoseStamped()
        msg.pose.position.x = 0 + np.random.uniform(-1, 1)
        msg.pose.position.y = 0 + np.random.uniform(-1, 1)
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    pose_pub = PosePub()

    rclpy.spin(pose_pub)

    pose_pub.destroy_node()
    rclpy.shutdown()