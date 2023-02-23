#!/usr/bin/env python3
"""
Author: Alex Sloot
University of Groningen
Last modified: 23-03-2023
"""
from typing import List
from landmark import Landmark


class DistanceOnlyEstimator:
    """Estimator for a landmark position using range only measurements."""

    def __init__(self, landmark: Landmark) -> None:
        """Initializer, needs a landmark object."""
        self.landmark = landmark

    def do_iteration(
        self,
        robot_x: float,
        robot_y: float,
        time_step: float,
        u: List[float],
        w: List[float],
    ) -> None:
        """Perform an iteration of measure, calculate and update."""
        self.measure(robot_x, robot_y)
        self.calculate(robot_x, robot_y, time_step, u, w)
        self.update()

    def measure(self, robot_x: float, robot_y: float) -> None:
        """Measure distance to landmark."""
        self.landmark.find_distances(robot_x, robot_y)

    def calculate(
        self,
        robot_x: float,
        robot_y: float,
        time_step: float,
        u: List[float],
        w: List[float],
    ) -> None:
        """Calculate the predicted robot and landmark positions."""
        self.process_data()
        self.predict(robot_x, robot_y, time_step, u, w)

    def process_data(self) -> None:
        """Process the measured data into useable inputs."""
        pass

    def predict(
        self,
        robot_x: float,
        robot_y: float,
        time_step: float,
        u: List[float],
        w: List[float],
    ) -> None:
        """Predict where the robot and landmark are using past estimate and new measurement."""
        self.landmark.update_estimate(robot_x, robot_y, time_step, u, w)

    def decide_movement(self, robot_x: float, robot_y: float, magnitude: float) -> None:
        """Decide how to move the robot based on the prediction"""
        z_est = self.landmark.get_z_estimate()
        z_distance = self.landmark.predict_distance(robot_x, robot_y)
        v = [z_est[0] / z_distance, z_est[1] / z_distance]
        w = [v[1], -v[0]]
        return [magnitude * w[0], magnitude * w[1]]

    def update(self) -> None:
        """Update the landmark and robot position estimations"""
        # potentially use a Kalman filter or other update rules
        pass

    def print(self) -> None:
        """Print the found and true landmark position."""
        self.landmark.print()
