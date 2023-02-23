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
    """Starts an empty Gazebo world from the existing terminal."""
    # -e can be replaced by --, but then it crashes
    subprocess.Popen(["gnome-terminal", "-e", "rosrun gazebo_ros gazebo"])
    time.sleep(3)
    close_message_print()


def close_message_print() -> None:
    print("-----------------------------------------------------------------")
    print("Only close the 'roscore' and 'gazebo' terminals using 'CTRL + C'.")
    print("-----------------------------------------------------------------")


if __name__ == "__main__":
    start_roscore()
    start_gazebo()
