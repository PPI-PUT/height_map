import numpy as np
from dataclasses import dataclass
import matplotlib.pyplot as plt
from height_map.utils import Point_3D, Step


@dataclass
class Pose_2D:
    x: float
    y: float
    yaw: float


class HeightMap(object):
    def __init__(
        self, steps: np.ndarray, y_length: float, x_length: float, step_size: float
    ):
        self.steps = steps
        self.y_length = y_length
        self.x_length = x_length
        self.step_size = step_size

    def add_step(self, points: np.ndarray):
        self.steps = np.append(self.steps, Step(points))

    def get_height_map(self, pose: Pose_2D):
        """
        Get height map around the pose
        """
        x = np.arange(
            - self.x_length / 2, self.x_length / 2 + self.step_size, self.step_size
        )
        y = np.arange(
            - self.y_length / 2, self.y_length / 2 + self.step_size, self.step_size
        )
        X, Y = np.meshgrid(x, y)

        # rotate the meshgrid by angle of yaw
        X_rot = X * np.cos(pose.yaw) - Y * np.sin(pose.yaw)
        Y_rot = X * np.sin(pose.yaw) + Y * np.cos(pose.yaw)

        X_rot += pose.x
        Y_rot += pose.y

        Z = np.zeros(X_rot.shape)

        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection="3d")

        for step in self.steps:
            for i in range(X_rot.shape[0]):
                for j in range(X_rot.shape[1]):
                    Z[i, j] += step.get_z(X_rot[i, j], Y_rot[i, j])

        # ax.scatter(X_rot, Y_rot, Z, c="b", marker="o", label="plane")
        # plt.show()

        return X_rot, Y_rot, Z


# if __name__ == "__main__":
#     h1 = 0.2
#     h2 = 0.4
#     step_1 = Step(
#         np.array(
#             [
#                 Point_3D(0, 0, h1),
#                 Point_3D(1, 0, h1),
#                 Point_3D(1, 1, h1),
#                 Point_3D(0, 1, h1),
#             ]
#         )
#     )
#     step_2 = Step(
#         np.array(
#             [
#                 Point_3D(1, 0, h2),
#                 Point_3D(2, 0, h2),
#                 Point_3D(2, 1, h2),
#                 Point_3D(1, 1, h2),
#             ]
#         )
#     )
#     steps = np.array([step_1, step_2])
#     height_map = HeightMap(steps, 5, 5, 0.1)
#     pose = Pose_2D(0, 0, 0.0)
#     height_map.get_height_map(pose)
