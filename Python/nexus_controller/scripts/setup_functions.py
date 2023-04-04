#!/usr/bin/env python3
"""
Author: Alex Sloot
University of Groningen
Last modified: 14-03-2023
"""
import sys
from distance_only_estimator import DistanceOnlyEstimator
from estimator import Estimator
from landmark import Landmark
from wsr_estimator import WSREstimator


def parse_bool_argument(strbool: str) -> bool:
    """Converts the boolean string input to a boolean value."""
    strbool = strbool.lower()
    if strbool == "false":
        return False
    elif strbool == "true":
        return True
    else:
        print("WARNING: simulation is not a valid input, choose 'True' or 'False'!")
        sys.exit()


def parse_float_argument(strfloat: str) -> float:
    """Converts the string inputs to float values."""
    try:
        a = float(strfloat)
        return a
    except ValueError:
        print(f"ERROR: '{strfloat}' is not a valid input!")
        sys.exit()


def pick_estimator(estimator_type: str, landmark: Landmark) -> Estimator:
    """Picks an estimator baed on the input type given."""
    if estimator_type == "WSR":
        estimator = WSREstimator(landmark)
    elif estimator_type == "DO":
        estimator = DistanceOnlyEstimator(landmark)
    else:
        print("Not a valid estimator!")
        sys.exit()
    return estimator
