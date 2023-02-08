import sys, time, math
from nexus_car import NexusCar, CONVERT_CRLF
from serial import SerialException


def create_a_nexus_car(port: str) -> NexusCar:
    """Creates and returns a NexusCar object."""

    try:
        nexus_car = NexusCar(
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
        sys.stderr.write("could not open port %r: %s\n" % (port, e))
        sys.exit(1)

    # quick test to make sure the Nexus is ready
    time.sleep(2)
    nexus_car.sendpacket(20, 0)
    nexus_car.getinfopacket()

    return nexus_car
