import rclpy
from rclpy.node import Node
from rclpy import qos
from geometry_msgs.msg import Point32, Pose, PoseStamped
from sensor_msgs.msg import PointCloud

import numpy as np

from height_map.utils import Point_3D, Step
from height_map.height_map import HeightMap, Pose_2D

from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Quaternion


def euler_from_quaternion_scipy(quaternion):
    rotation = R.from_quat(quaternion)  # Input quaternion [x, y, z, w]
    return rotation.as_euler("xyz")


# point 1 
#   position:
#     x: -0.3770383894443512
#     y: -0.31675881147384644
#     z: 0.3471011817455292



# point 2
# pose:
#   position:
#     x: -0.42784810066223145
#     y: 0.8352770209312439
#     z: 0.35699042677879333


# point 3
#   position:
#     x: -1.1617645025253296
#     y: 0.7892267107963562
#     z: 0.3541583716869354



# point 4
#   position:
#     x: -1.1356751918792725
#     y: -0.3501853942871094
#     z: 0.34096768498420715


class HeightMapNode(Node):
    def __init__(self):
        super().__init__("height_map_node")
        self.get_logger().info("Height Map Node has been started")

        self.height_map = HeightMap(
            steps=np.array(
                [
                    Step(
                        np.array(
                            [
                                Point_3D(-0.3770383894443512, -0.31675881147384644, 0.3471011817455292),
                                Point_3D(-0.42784810066223145, 0.8352770209312439, 0.35699042677879333),
                                Point_3D(-1.1617645025253296, 0.7892267107963562, 0.3541583716869354),
                                Point_3D(-1.1356751918792725, -0.3501853942871094, 0.34096768498420715),
                            ]
                        )
                    )
                ]
            ),
            y_length=1.0,
            x_length=1.6,
            step_size=0.1,
        )

        self.pose_sub = self.create_subscription(
            PoseStamped, "optitrack/rigid_body_0", self.pose_callback, qos.qos_profile_sensor_data
        )

        self.height_map_pub = self.create_publisher(
            PointCloud, "/height_map", qos.qos_profile_sensor_data
        )

    def pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        yaw = euler_from_quaternion_scipy(
            [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]
        )[2] 
        print(f"x: {x}, y: {y}, yaw: {yaw}")

        pose = Pose_2D(x=x, y=y, yaw=yaw)
        X_rot, Y_rot, Z = self.height_map.get_height_map(pose)

        point_cloud = PointCloud()
        point_cloud.header.frame_id = "odom"
        point_cloud.header.stamp = self.get_clock().now().to_msg()
        for i in range(X_rot.shape[0]):
            for j in range(X_rot.shape[1]):
                point = Point32()
                point.x = X_rot[i, j]
                point.y = Y_rot[i, j]
                point.z = Z[i, j]
                point_cloud.points.append(point)

        self.height_map_pub.publish(point_cloud)


def main(args=None):
    rclpy.init(args=args)
    height_map_node = HeightMapNode()
    rclpy.spin(height_map_node)
    rclpy.shutdown()





