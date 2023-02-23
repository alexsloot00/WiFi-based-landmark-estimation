#!/usr/bin/env python3
"""
Author: Alex Sloot
University of Groningen
Last modified: 23-03-2023
"""

# Some type of base class, protocol possibly?
class Estimator:
    def measure(self):
        """Measure distance to landmark."""

    def calculate(self):
        """Calculate the predicted robot and landmark positions."""

    def process_data(self):
        """Process the measured data into useable inputs."""

    def predict(self):
        """Predict where the robot and landmark are using past estimate and new measurement."""

    def decide(self):
        """Decide how to act based on the prediction"""

    def update(self):
        """Update the landmark and robot position estimations"""
