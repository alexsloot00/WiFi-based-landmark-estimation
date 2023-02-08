#!/usr/bin/env python3
import sys
from distance_only_estimator import DistanceOnlyEstimator
from landmark import Landmark
from create_nexus_car import create_a_nexus_car


def main() -> None:
    """Run the Nexus robot landmark localization."""

    # change the port if needed
    port = "/dev/ttyUSB0"
    nexus_car = create_a_nexus_car(port)

    # create a landmark, initialize on robot position (0,0)
    landmark = Landmark(theta=0)
    landmark.initialize(0, 0)

    # choose an estimator and assign to the nexus_car
    estimator = DistanceOnlyEstimator(landmark)
    nexus_car.give_DO_estimator(estimator)

    # choose what to do
    # nexus_car.move_square()
    nexus_car.start()

    # stop the system
    nexus_car.stop()
    sys.exit()


if __name__ == "__main__":
    main()
