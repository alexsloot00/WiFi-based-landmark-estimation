#!/usr/bin/env python3


import sys, time
from serial import SerialException

from nexus_car import NexusCar, CONVERT_CRLF
from terminal_functions import start_gazebo


def create_a_nexus_car(name: str, port: str, simulation: bool) -> NexusCar:
    """Creates and returns a NexusCar object."""

    nexus_car = NexusCar(
        velocity_magnitude=0.1,
        time_step=0.05,
        simulation=simulation,
    )

    if simulation:
        try:
            start_gazebo()
            nexus_car.init_simulation(model_name=name)
        except Exception as e:  # find a proper Gazebo error
            sys.stderr.write(f"could not open Gazebo. {e}")
        return nexus_car

    try:
        nexus_car.init_connection(
            port=port,
            baudrate=28800,
            parity="N",
            rtscts=False,
            xonxoff=False,
            echo=False,
            convert_outgoing=CONVERT_CRLF,
            repr_mode=0,
        )

    except SerialException as e:
        sys.stderr.write("Could not open port %r: %s\n" % (port, e))
        sys.exit(1)

    # quick test to make sure the Nexus is ready
    time.sleep(2)
    nexus_car.sendpacket(20, 0)
    nexus_car.getinfopacket()

    return nexus_car
