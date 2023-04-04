#!/usr/bin/env python3
"""
Author: Alex Sloot
University of Groningen
Last modified: 14-03-2023
"""
import math


def map_to_two_pi(value: float) -> float:
    """Maps a value in radians to [-pi, pi]."""
    if value > 2 * math.pi:
        return value - 2 * math.pi
    elif value < -2 * math.pi:
        return value + 2 * math.pi
    else:
        return value
