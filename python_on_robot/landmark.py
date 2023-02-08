#!/usr/bin/env python3


import math
from dataclasses import dataclass, field
from numpy import sign
from typing import List

from helper_functions import map_to_two_pi


@dataclass
class Landmark:
    """A landmark object with estimated theta, distance,x,y and true x,y.
    An initial theta value is needed to create a landmark object."""

    theta: float
    gain: float = 10.0
    pr_distance: float = field(init=False)
    x: float = field(init=False)
    y: float = field(init=False)
    _x_star: float = 6.0
    _y_star: float = 6.5
    # _distance is not constant -> function

    def initialize(self, robot_x: float, robot_y: float) -> None:
        """Give initial values to non-initiated distance,x,y."""
        # uses the actual distance as first d value.
        self.pr_distance = self._measure_distance(robot_x, robot_y)  # measurement
        self.m_distance = self._measure_distance(robot_x, robot_y)  # should be unknown
        self._update_xy(robot_x, robot_y)  # uses _distance, should use distance
        print(f"Initial landmark prediction values: x={self.x:.2f}, y={self.y:.2f}")

    def find_distances(self, robot_x: float, robot_y: float) -> None:
        """Calulate and store the new predicted distance."""
        self.pr_distance = self.predict_distance(robot_x, robot_y)
        self.m_distance = self._measure_distance(robot_x, robot_y)

    def update_estimate(
        self,
        robot_x: float,
        robot_y: float,
        time_step: float,
        u: List[float],
        w: List[float],
    ) -> None:
        """Update the estimation for the landmark."""
        self.update_theta(time_step, u, w)
        self._update_xy(robot_x, robot_y)

    def update_theta(self, time_step: float, u: List[float], w: List[float]) -> None:
        """Update the estimated angle theta to the landmark."""
        theta_update = pow(self.m_distance, 2) - pow(self.pr_distance, 2)
        u_w = u[0] * w[0] + u[1] * w[1]
        theta = self.theta + time_step * u_w + self.gain * sign(u_w) * theta_update
        self.theta = map_to_two_pi(theta)

    def update_xy(self, robot_x: float, robot_y: float) -> None:
        """Update the x and y coordinates of the landmark."""
        # update the xy based on new theta and pr_d or m_d????
        self._update_xy(robot_x, robot_y)

    def _update_theta(self, robot_x: float, robot_y: float) -> None:
        """Update the estimated theta value of the landmark."""
        self.theta = math.atan2((self.y - robot_y), (self.x - robot_x)) + self.gain * (
            self.pr_distance - self.m_distance
        )

    def _update_xy(self, robot_x: float, robot_y: float) -> None:
        """Update the x,y coordinates of the landmark."""
        # z_est = distance * [math.cos(theta), math.sin(theta)]
        # because distance has 0 error, we only use 'measurement'
        # otherwise use estimation 'y_hat', or a combination.
        z_est = self.get_z_estimate()
        self.x = robot_x + z_est[0]
        self.y = robot_y + z_est[1]

    def get_z_estimate(self) -> List[float]:
        """The vector between estimated landmark and robot."""
        return [
            self.m_distance * math.cos(self.theta),
            self.m_distance * math.sin(self.theta),
        ]

    def predict_distance(self, robot_x: float, robot_y: float) -> float:
        """Euclidean distance between robot x,y and estimated landmark x,y."""
        # PREDICTION OF MEASUREMENT
        # self.pr_distance = y_pred = norm(l_est - p)
        return math.sqrt(pow((self.x - robot_x), 2) + pow((self.y - robot_y), 2))

    def _measure_distance(self, robot_x: float, robot_y: float) -> float:
        """Euclidean distance between robot x,y and true landmark x,y."""
        # ACTUAL MEASUREMENT
        # self.m_distance = y = norm(l_star - p)
        return math.sqrt(
            pow((self._x_star - robot_x), 2) + pow((self._y_star - robot_y), 2)
        )

    def _theta_star(self, robot_x: float, robot_y: float) -> float:
        """The angle theta between robot x,y and landmark x,y."""
        return math.atan2((self._y_star - robot_y), (self._x_star - robot_x))

    def print(self) -> None:
        print(f"Found theta,d values area {self.theta:.2f}, {self.pr_distance:.2f}")
        print(f"Found x,y landmark values are {self.x:.2f}, {self.y:.2f}")
        print(f"True x,y landmark values are {self._x_star:.2f}, {self._y_star:.2f}")


def test() -> None:
    """Only ran for testing."""
    a = Landmark(0.0)
    a.initialize(4, 8)
    for i in range(25):
        x, y = 4 + i / 100, 8 - i / 100
        u = [1 / 100, 1 / 100]
        w = [u[0] / 100, u[1] / 100]
        a.find_distances(x, y)
        a.update_estimate(x, y, 0.1, u, w)
    a.print()


if __name__ == "__main__":
    test()
