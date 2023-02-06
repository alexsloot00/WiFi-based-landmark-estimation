#!/usr/bin/env python3

import sys, time
import new_carcomm

if __name__ == "__main__":

    try:
        # voorlopig vast
        port = "/dev/ttyUSB0"
        baudrate = 28800
        parity = "N"
        rtscts = False
        xonxoff = False
        echo = False
        convert_outgoing = new_carcomm.CONVERT_CRLF
        repr_mode = 0

        # create carcomm CarControl object
        carcon = new_carcomm.CarControl(
            port,
            baudrate,
            parity,
            rtscts,
            xonxoff,
            echo,
            convert_outgoing,
            repr_mode,
        )
        time.sleep(2)  # de nexus robot is niet zo snel..
        carcon.sendpacket(20, 0)
        carcon.getinfopacket()

        # move a square
        carcon.setspeed(0.1, 0, 0)
        time.sleep(3)
        carcon.setspeed(0, 0.1, 0)
        time.sleep(3)
        carcon.setspeed(-0.1, 0, 0)
        time.sleep(3)
        carcon.setspeed(0, -0.1, 0)
        time.sleep(3)
        carcon.setspeed(0, 0, 0)

    except new_carcomm.serial.SerialException as e:
        sys.stderr.write("could not open port %r: %s\n" % (port, e))
        sys.exit(1)
