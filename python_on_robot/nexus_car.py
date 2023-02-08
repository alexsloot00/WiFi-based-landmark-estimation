#!/usr/bin/env python3

import sys, time, traceback, serial
from typing import List
from struct import pack, unpack
from distance_only_estimator import DistanceOnlyEstimator
from wsr_estimator import WSREstimator

CONVERT_CRLF = 2
CONVERT_CR = 1
CONVERT_LF = 0
NEWLINE_CONVERISON_MAP = ("\n", "\r", "\r\n")


class NexusCar:
    """Object containing connection and movement options for a NexusCar."""

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
        velocity_magnitude=0.1,
        time_step=0.05,
    ):
        """Initialize the connection and inputs for a NexusCar object."""
        self.serial = serial.Serial(
            port, baudrate, parity=parity, rtscts=rtscts, xonxoff=xonxoff, timeout=1
        )
        self.echo = echo
        self.repr_mode = repr_mode
        self.convert_outgoing = convert_outgoing
        self.newline = NEWLINE_CONVERISON_MAP[self.convert_outgoing]
        self.dtr_state = True
        self.rts_state = True
        self.break_state = False
        self.velocity_magnitude = velocity_magnitude
        self.time_step = time_step

        self.post_init()

    def post_init(self) -> None:
        """Quick check and set the coordinate system."""
        self.x = 0
        self.y = 0
        self.previous_move = [0.0, 0.0, 0.0]
        print(f"Nexus Car is properly intialized and ready for use.")

    def stop(self) -> None:
        """Set movement to 0 and close the connection."""
        self.setspeed(0, 0, 0)
        self.serial.close()

    def give_DO_estimator(self, estimator: DistanceOnlyEstimator) -> None:
        """Uses a distance-only estimator."""
        self.estimator = estimator

    def give_WSR_estimator(self, estimator: WSREstimator) -> None:
        """Uses a WiFi-only WSR toolbox estimator."""
        self.estimator = estimator

    def start(self) -> None:
        """Move around and locate a landmark."""
        for _ in range(50):
            self.measure()
            self.calculate()
            time.sleep(self.time_step)
        self.estimator.print()

    def measure(self):
        """Measure wheel rotations for own x, y, theta position."""
        # self.measurement =  delta_x, delta_y, delta_theta (how to obtain?)
        # self.x += delta_x, self.y += delta_y, self.theta += delta_theta
        self.x = self.x + self.velocity_magnitude * self.previous_move[0]
        self.y = self.y + self.velocity_magnitude * self.previous_move[1]

    def calculate(self) -> None:
        """Calculate the predicted robot and landmark positions."""
        self.process_data()
        self.predict()
        movement = self.decide()
        self.act(movement)

    def process_data(self) -> None:
        """Process the measured data into useable inputs."""
        # probably nothing to process, already in proper formatfrom
        pass

    def predict(self) -> None:
        """Predict where the robot and landmark are using past estimate and new measurement."""
        u = [self.previous_move[0], self.previous_move[1]]
        w = [u[0] / self.velocity_magnitude, u[1] / self.velocity_magnitude]
        self.estimator.do_iteration(self.x, self.y, self.time_step, u, w)

    def decide(self) -> List[float]:
        """Decide how to act based on the prediction"""
        # move orthogonally to landmark?
        return self.estimator.decide_movement(self.x, self.y, self.velocity_magnitude)

    def update(self) -> None:
        """Update the landmark and robot position estimations"""
        # already done in measure and predict (first update robot_pos before doing any calculations)
        pass

    def act(self, movement: List[float]) -> None:
        """Use control algorithm and model parameters to translate into action"""
        # robot_pos_predict = [self.x, self.y]
        # landmark_pos_predict = self.estimator.xy()
        # robot_pos_measure = []  # no idea
        # landmark_pos_measure = []  # no idea
        x = movement[0]
        y = movement[1]
        rotation = 0  # not provided yet
        self.setspeed(x, y, rotation)
        self.previous_move = [x, y, rotation]

    def move_square(self) -> None:
        """Move the Nexus in a square trajectory."""
        self.setspeed(self.velocity_magnitude, 0, 0)
        time.sleep(3)
        self.setspeed(0, self.velocity_magnitude, 0)
        time.sleep(3)
        self.setspeed(-self.velocity_magnitude, 0, 0)
        time.sleep(3)
        self.setspeed(0, -self.velocity_magnitude, 0)
        time.sleep(3)
        self.setspeed(0, 0, 0)

    #### Send commands
    def sendwheelscommand(self, frontleft, backleft, frontright, backright):
        """Sends the commands to the raspberry pi, which controls the motors."""
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
        """Sets the speed of the Nexus in x (forward), y(sideways) directions and rotates by rot."""
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

    #### stuff for checking etc

    def getinfopacket(self) -> None:
        """Check connection with Raspberry Pi from Nexus."""
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
            print("No info packet")
            traceback.print_exc(file=sys.stdout)

    def sendpacket(self, command, par) -> None:
        """Send a data packet to the Raspberry Pi on the Nexus."""
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


""" 
    def move(self, joystick_x, joystick_y):
        angle = math.atan2(joystick_y, joystick_x)
        magnitude = math.sqrt(joystick_x**2.0 + joystick_y**2.0)

        power_fr_bl = magnitude * math.sin(angle - 1 / 4 * math.pi)
        power_fl_br = magnitude * math.sin(angle + 1 / 4 * math.pi)

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
