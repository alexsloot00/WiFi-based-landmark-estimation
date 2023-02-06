#!/usr/bin/env python3

#  To control Nexus robot car
#          Via /dev/ttyUSB?
#
#  SB-2022  aanpassing tbv python 3

""" 
IMPORTANT to run in python 3:
run 'pip3 list'
if 'serial' is in the list run
pip3 uninstall serial
if 'pyserial' is not in the list run
pip3 install pyserial

serial and pyserial are both imported as
import serial
but pyserial is the one we need and 
serial interferes with this import
https://github.com/espressif/esptool/issues/269 (halverwege pagina)
"""

import sys, os, serial, time, traceback
from struct import pack, unpack
import numpy as np
import re


def key_description(character):
    """generate a readable description for a key"""
    ascii_code = ord(character)
    if ascii_code < 32:
        return "Ctrl+%c" % (ord("@") + ascii_code)
    else:
        return repr(character)


# voorlopig alleen posix
if os.name == "posixnee":
    import termios, sys, os

    class Console:
        def __init__(self):
            self.fd = sys.stdin.fileno()

        def setup(self):
            self.old = termios.tcgetattr(self.fd)
            new = termios.tcgetattr(self.fd)
            #            new[3] = new[3] & ~termios.ICANON & ~termios.ECHO & ~termios.ISIG
            new[3] = new[3] & ~termios.ICANON  # & ~termios.ISIG
            new[6][termios.VMIN] = 1
            new[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, new)

        #  Uiteindelijk willen we meer tegelijk lezen..
        #   zeker als er grotere hoeveelheden data verstuurd worden door het target
        def getkey(self):
            c = os.read(self.fd, 1)
            return c

        def cleanup(self):
            termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.old)

    console = Console()

    def cleanup_console():
        console.cleanup()

    console.setup()
    sys.exitfunc = cleanup_console  # terminal modes have to be restored on exit...

# else:
#    raise NotImplementedError( "Sorry no implementation for your platform (%s) available." % sys.platform)


CONVERT_CRLF = 2
CONVERT_CR = 1
CONVERT_LF = 0
NEWLINE_CONVERISON_MAP = ("\n", "\r", "\r\n")
LF_MODES = ("LF", "CR", "CR/LF")

REPR_MODES = ("raw", "some control", "all control", "hex")


class CarControl:
    def __init__(
        self,
        port,
        baudrate,
        parity,
        rtscts,
        xonxoff,
        echo=False,
        convert_outgoing=CONVERT_CRLF,
        repr_mode=0,
    ):
        print(f"The python version is {sys.version}")
        print(f"The serial version is {serial.__version__}")
        print("Creating the serial")
        self.serial = serial.Serial(
            port, baudrate, parity=parity, rtscts=rtscts, xonxoff=xonxoff, timeout=1
        )
        print("Serial created")
        self.echo = echo
        self.repr_mode = repr_mode
        self.convert_outgoing = convert_outgoing
        self.newline = NEWLINE_CONVERISON_MAP[self.convert_outgoing]
        self.dtr_state = True
        self.rts_state = True
        self.break_state = False

        # Het lijkt er op of er na een power up van de Nexus twee characters (waarde 0) gezonden worden. FTDI chip?
        # De writer pas wat laten doen als we daar geen last meer van hebben.
        tmp = self.serial.read(1)
        # print 'pretmp', len(tmp)
        # if len(tmp) > 0:
        #    print ord(tmp[0])
        time.sleep(1)
        tmp = self.serial.read(1)
        # print 'prettmp', len(tmp)
        # if len(tmp) > 0:
        #    print ord(tmp[0])

    def stop(self):
        print("Waar is de stop?")
        self.serial.close()

    def getinfopacket(self):
        try:
            d = self.serial.read(24)
            print(len(d))  # , ord(d[0])
            if (d[0] != 2) or (d[23] != 3):  # STX and ETX
                print("Packet framing error!")
            else:
                print("New info packet received.")
                if d[1] == 1:  # scope data packet (try that first)
                    status_control = d[3]
                    error_code = d[4]
                    status_car = d[5]
                    # (sensor,)   = unpack('h', d[10:12]);
                    (speed,) = unpack("h", d[6:8])
                    (debtime,) = unpack("h", d[10:12])
                    # we gebruiken dit in de write thread en bij autoreply.
                    # evt. korte lock bij uitpakken data
                    waitreq = 1
                else:
                    print("Packet type %x received, not implemented.") % d[1]
                print(
                    "Info packet received with error code ",
                    error_code,
                    " status car",
                    status_car,
                    "speed",
                    speed,
                    "debtime",
                    debtime,
                )
        except:
            print("Wanneer hier?")
            traceback.print_exc(file=sys.stdout)

    """
    Packets are 8 bytes.
         { stx, command, par's ..., etx }
             parameters dependant upon the command

    Nexus robot functions of type void(*function[11])(unsigned int speedMMPS) :
      {goAhead, turnLeft, turnRight, rotateRight, rotateLeft, allStop,  backOff,  upperLeft, lowerLeft, upperRight, lowerRight};
      {  0,        1,        2,         3,            4,         5,        6,         7,         8,         9,         10     }
    this wil go in command.
    Parameter speedMMPS in par

    -700 < speedMMPS < 700
    speedMMPS = 200  komt overeen met 0.21 m/sec
      maximale snelheid 0.735 m/sec
      van m/s naar speedMMPS : maal 952


    Other commands:
    12: setWheelspeedcommand (set speed of 4 wheels)
    13: set speedMMPS on Nexus
    20: ask info packet a 24 byte packet will be received
    21: autoshutdown off (default)
    22: autoshutdown on
    23: autoreply off (default)
    24: autoreply on. reply with a infopacket            carcon.setspeed( 0.2, 0.0, 0.0)

    25: start standalone?
    26: stop standalone
    27: keep alive
    28: repeat mode info packets off (default)
    29: repeat mode info packete on

    """

    def sendpacket(self, command, par):
        outpkt = bytearray(16)
        outpkt[0] = 2  # STX
        outpkt[1] = command
        outpkt[2:4] = pack("h", par)  # ' h'  == dec 104
        outpkt[15] = 3  # ETX
        self.serial.write(outpkt)  # send packet
        print(
            "sendpacket",
            outpkt[0],
            outpkt[1],
            outpkt[2],
            outpkt[3],
            outpkt[4],
            outpkt[5],
            outpkt[6],
            " ... ",
            outpkt[15],
        )

    def sendwheelscommand(self, frontleft, backleft, frontright, backright):
        outpkt = bytearray(16)
        outpkt[0] = 2  # STX
        outpkt[1] = 12  # setwheels command
        outpkt[2:4] = pack("h", int(frontleft))
        outpkt[4:6] = pack("h", int(backleft))
        outpkt[6:8] = pack("h", int(frontright))
        outpkt[8:10] = pack("h", int(backright))

        outpkt[15] = 3  # ETX
        self.serial.write(outpkt)  # send packet
        # print 'outpkt: ', unpack('h', outpkt[2:4]), unpack('h', outpkt[4:6]), unpack('h', outpkt[6:8]), unpack('h',outpkt[8:10])

    def setspeed(self, linx, liny, rot):
        # lineaire x and y (m/sec) and rotatie (rad/sec) snelheid
        # convert to nexus (calibrated with nexus1)
        speedx = linx * -1024
        speedy = liny * -1024
        rot = rot * 300
        self.sendwheelscommand(
            speedx - speedy - rot,
            speedx + speedy - rot,
            speedx - speedy + rot,
            speedx + speedy + rot,
        )


""" 
    def move(self, joystick_x, joystick_y):
        angle = math.atan2(joystick_y, joystick_x)
        magnitude = math.sqrt(joystick_x**2.0 + joystick_y**2.0)

        power_fr_bl = magnitude * math.sin(angle - 1 / 4 * math.pi)
        power_fl_br = magnitude * math.sin(angle + 1 / 4 * math.pi)
"""
