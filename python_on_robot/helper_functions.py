import math


def map_to_two_pi(value: float) -> float:
    """Maps a value in radians to [-pi, pi]."""
    if value > 2 * math.pi:
        return value - 2 * math.pi
    elif value < -2 * math.pi:
        return value + 2 * math.pi
    else:
        return value
