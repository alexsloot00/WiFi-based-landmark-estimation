#!/usr/bin/env python3
"""
Author: Alex Sloot
University of Groningen
Last modified: 23-03-2023
"""

import math
from estimator import Estimator
from landmark import Landmark
from typing import List


class WSREstimator(Estimator):
    def __init__(self, landmark: Landmark) -> None:
        """Initializer, needs a landmark object."""
        self.landmark = landmark
        self.previous_angle = 0.0
        self.prev_time_passed = 0.0

    def do_iteration(
        self,
        robot_x: float,
        robot_y: float,
        time_step: float,
        u: List[float],
        w: List[float],
    ):
        pass

    def measure(self, robot_x: float, robot_y: float) -> None:
        """Measure distance to landmark."""
        pass

    def calculate(
        self,
        robot_x: float,
        robot_y: float,
        time_step: float,
        u: List[float],
        w: List[float],
    ) -> None:
        """Calculate the predicted robot and landmark positions."""
        pass

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
        pass

    def decide_movement(
        self,
        robot_x: float,
        robot_y: float,
        magnitude: float,
        time_passed: float,
        move: str = "circle",
    ) -> List[float]:
        """Decide how to act based on the prediction"""
        # moving right needs * 1.33 as the motors to move right are weaker!
        print(time_passed)
        move = move.lower()
        if move == "circle":
            circle_radius = magnitude
            # angle based on 10 seconds total time
            angle = time_passed / 10 * math.radians(360)
            print(angle)
            w = (angle - self.previous_angle) / (
                time_passed - self.prev_time_passed
            )  # derivative of angle increase
            xdot = -circle_radius * w * math.sin(angle)
            ydot = circle_radius * w * math.cos(angle)
            if ydot > 0:
                ydot *= 1.33
            self.previous_angle = angle
            self.prev_time_passed = time_passed
            return [xdot, ydot]
        elif move == "forward":  # forward is negative!
            return [-magnitude, 0]
        elif move == "backward":  # backward is positive!
            return [magnitude, 0]
        elif move == "right":  # right is positive!
            return [0, 1.33 * magnitude]
        elif move == "left":  # left is negative!
            return [0, -magnitude]
        else:
            print("Not a valid movement option! Remain standing still")
            return [0, 0]

    def update(self) -> None:
        """Update the landmark and robot position estimations"""
        pass

    def print(self) -> None:
        print("printing")


def test(time_passed):
    circle_radius = 1  # meters
    # angle based on 10 seconds total time
    angle = time_passed / 10 * math.radians(360)
    w = 1  # derivative of angle increase
    xdot = -circle_radius * w * math.sin(angle)
    ydot = circle_radius * w * math.cos(angle)
    return [xdot, ydot]


if __name__ == "__main__":
    a = test(0)
    print(a)
    a = test(2.5)
    print(a)
    a = test(5)
    print(a)
    a = test(7.5)
    print(a)
    a = test(10)
    print(a)
