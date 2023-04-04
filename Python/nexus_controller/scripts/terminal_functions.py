#!/usr/bin/env python3
"""
Author: Alex Sloot
University of Groningen
Last modified: 23-03-2023
"""

import subprocess
import time
import rospy


def start_roscore() -> None:
    """If no roscore is running, start a roscore in a new terminal."""
    if not rospy.core.is_initialized():
        subprocess.Popen(["gnome-terminal", "--", "roscore"])
        time.sleep(3)


def start_gazebo() -> None:
    """Starts an empty Gazebo world from a new terminal."""
    subprocess.Popen(["gnome-terminal", "-e", "rosrun gazebo_ros gazebo"])
    time.sleep(3)
    close_message_print()


def close_message_print() -> None:
    print("-----------------------------------------------------------------")
    print("Only close the 'roscore' and 'gazebo' terminals using 'CTRL + C'.")
    print("-----------------------------------------------------------------")


# def setup_WSR() -> None:
#     """Sets the correct RX stuff."""
#     subprocess.Popen(
#         ["gnome-terminal", "-e", "sudo ./WSR-WifiDriver/setup.sh 59 108 HT20"]
#     )


# # "sudo ./WSR-WifiDriver/setup.sh 59 108 HT20 && sudo ~/WSR-Toolbox-linux-80211n-csitool-supplementary/netlink/log_to_file csi_rx.dat",


# def start_WSR_transmitting() -> None:
#     """Starts to transmit data."""
#     subprocess.Popen(
#         [
#             "gnome-terminal",
#             "--",
#             "sudo ~/WSR-Toolbox-linux-80211n-csitool-supplementary/injection/random_packets_two_antenna 1000 57 1 7000",
#         ]
#     )


if __name__ == "__main__":
    start_roscore()
    start_gazebo()
