#!/usr/bin/env python3
"""
Author: Alex Sloot
University of Groningen
Last modified: 14-03-2023
"""

import sys, time, argparse
from landmark import Landmark
from create_nexus_car import create_a_nexus_car
from setup_functions import parse_bool_argument, parse_float_argument, pick_estimator
from terminal_functions import start_roscore


def main() -> None:
    """Run the Nexus robot landmark localization."""
    # parsing optional arguments from shell
    parser = argparse.ArgumentParser(description="Add optional parameters")
    parser.add_argument(
        "-simulation",
        "--simulation",
        help="choose: True/False, i.e. -simulation False",
        required=False,
        default="True",
    )
    parser.add_argument(
        "-estimator",
        "--estimator",
        help="choose WSR/DO (distance-only), i.e. -estimator DO",
        required=False,
        default="WSR",
    )
    parser.add_argument(
        "-name",
        "--name",
        help="choose: a name, i.e. -name nexus_car",
        required=False,
        default="nexus_car",
    )
    parser.add_argument(
        "-port",
        "--port",
        help="choose: a port, i.e. -port /dev/ttyUSB0",
        required=False,
        default="/dev/ttyUSB0",
    )
    parser.add_argument(
        "-velmag",
        "--velmag",
        help="Choose velocity magnitude: i.e. -velmag 0.1",
        required=False,
        default=0.1,
    )
    parser.add_argument(
        "-timestep",
        "--timestep",
        help="Choose a timestep in seconds: i.e. -timestep 0.05",
        required=False,
        default=0.05,
    )
    parser.add_argument(
        "-move",
        "--move",
        help="Choose: circle, forward, backward, right, left: i.e. -move circle",
        required=False,
        default="circle",
    )
    parser.add_argument(
        "-runtime",
        "--runtime",
        help="Choose how long to run in seconds: i.e. -runtime 10.0",
        required=False,
        default=10.0,
    )

    # parsing the arguments from string to their repective types
    argument = parser.parse_args()
    simulation = parse_bool_argument(argument.simulation)
    estimator_type = argument.estimator
    name = argument.name
    port = argument.port
    velocity_magnitude = parse_float_argument(argument.velmag)
    time_step = parse_float_argument(argument.timestep)
    move = argument.move
    runtime = parse_float_argument(argument.runtime)

    # Note: start one manually if using ssh access
    if simulation:
        start_roscore()

    # create an instance of the nexus car
    nexus_car = create_a_nexus_car(
        simulation=simulation,
        name=name,
        port=port,
        velocity_magnitude=velocity_magnitude,
        time_step=time_step,
        move=move,
        runtime=runtime,
    )

    # create a landmark, initialize on robot position (0,0)
    landmark = Landmark(theta=-1.6, simulation=simulation)
    landmark.initialize(0, 0)

    # choose an estimator and assign to the nexus_car
    estimator = pick_estimator(estimator_type, landmark)
    nexus_car.give_estimator(estimator)

    # choose what to do
    # nexus_car.move_demo_square()
    print("Wait...")
    time.sleep(4)
    print("Starting in 1 second.")
    time.sleep(1)
    nexus_car.start()

    # stop the system
    # nexus_car.stop()
    sys.exit()


if __name__ == "__main__":
    main()
