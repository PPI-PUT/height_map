import numpy as np
from dataclasses import dataclass
import matplotlib.pyplot as plt


@dataclass
class Point_3D:
    x: float
    y: float
    z: float


class Step(object):
    # argument is array of 4 Point_2D objects
    def __init__(self, points: np.ndarray):
        self.points = points
        [self.a, self.b, self.c] = self.calculate_surface()

    def calculate_surface(self):
        """
        Taka points and perform linear regression to calculate surface of step
        Equation of plane z = ax + by + c
        """
        x = np.array([point.x for point in self.points])
        y = np.array([point.y for point in self.points])
        z = np.array([point.z for point in self.points])

        A = np.vstack([x, y, np.ones(len(x))]).T
        a, b, c = np.linalg.lstsq(A, z, rcond=None)[0]
        print("Surface equation: z = {}x + {}y + {}".format(a, b, c))
        return a, b, c

    def get_z(self, x: float, y: float):
        """
        Check if point is between 4 points and return z value
        """
        is_in = self.is_inside(self.points, x, y)

        if is_in:
            return self.a * x + self.b * y + self.c
        else:
            return 0

    def is_inside(self, polygon: np.ndarray, x: float, y: float):
        x_intersections = 0
        point_x, point_y = x, y
        n = len(polygon)

        for i in range(n):
            p1_x, p1_y = polygon[i].x, polygon[i].y
            p2_x, p2_y = polygon[(i + 1) % n].x, polygon[(i + 1) % n].y

            if min(p1_y, p2_y) < point_y <= max(p1_y, p2_y):
                if p1_y != p2_y:
                    xinters = (point_y - p1_y) * (p2_x - p1_x) / (p2_y - p1_y) + p1_x
                else:
                    xinters = p1_x

                if point_x <= xinters:
                    x_intersections += 1

        return x_intersections % 2 != 0

    def plot(self):
        """
        Plot step
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")

        x = [point.x for point in self.points]
        y = [point.y for point in self.points]
        z = [point.z for point in self.points]

        ax.scatter(x, y, z, c="r", marker="o", label="anchor points")

        x = np.linspace(-5, 5, 100)
        y = np.linspace(-5, 5, 100)
        X, Y = np.meshgrid(x, y)
        Z = np.empty(X.shape)
        for i in range(X.shape[0]):
            for j in range(X.shape[1]):
                Z[i, j] = self.get_z(X[i, j], Y[i, j])

        ax.scatter(X, Y, Z, c="b", marker="o", label="plane")

        plt.show()
