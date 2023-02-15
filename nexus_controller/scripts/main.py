#!/usr/bin/env python3
import sys
from distance_only_estimator import DistanceOnlyEstimator
from landmark import Landmark
from create_nexus_car import create_a_nexus_car
from terminal_functions import start_roscore
import time


def main() -> None:
    """Run the Nexus robot landmark localization."""

    simulation = True
    port = "/dev/ttyUSB0"
    name = "nexus_car"

    # start a rosmaster
    start_roscore()

    # create an instance of the nexus car
    nexus_car = create_a_nexus_car(name=name, port=port, simulation=simulation)

    # create a landmark, initialize on robot position (0,0)
    landmark = Landmark(theta=-1.6, simulation=simulation)
    landmark.initialize(0, 0)

    # choose an estimator and assign to the nexus_car
    estimator = DistanceOnlyEstimator(landmark)
    nexus_car.give_DO_estimator(estimator)

    # choose what to do
    # nexus_car.move_square()
    time.sleep(5)
    nexus_car.start()

    # stop the system
    nexus_car.stop()
    sys.exit()


if __name__ == "__main__":
    main()
