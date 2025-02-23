from typing import Tuple

import numpy as np


def delta_phi(ticks: int, prev_ticks: int, resolution: int) -> Tuple[float, float]:
    """
    Args:
        ticks: Current tick count from the encoders.
        prev_ticks: Previous tick count from the encoders.
        resolution: Number of ticks per full wheel rotation returned by the encoder.
    Return:
        dphi: Rotation of the wheel in radians.
        ticks: current number of ticks.
    """

    delta_ticks = ticks - prev_ticks
    
    # The resolution is N_tot. In order to get the amount of rotation per tick in radians, we do 2pi/resolution. This is our alpha
    rotation_per_tick = 2 * np.pi / resolution

    # We also have the equation dphi = N_k * alpha where N_k is the number of ticks in the most recent tiem interval, so delta_ticks
    dphi = delta_ticks * rotation_per_tick
    # ---
    return dphi, ticks


def pose_estimation(
    R: float,
    baseline: float,
    x_prev: float,
    y_prev: float,
    theta_prev: float,
    delta_phi_left: float,
    delta_phi_right: float,
) -> Tuple[float, float, float]:

    """
    Calculate the current Duckiebot pose using the dead-reckoning model.

    Args:
        R:                  radius of wheel (both wheels are assumed to have the same size) - this is fixed in simulation,
                            and will be imported from your saved calibration for the real robot
        baseline:           distance from wheel to wheel; 2L of the theory
        x_prev:             previous x estimate - assume given
        y_prev:             previous y estimate - assume given
        theta_prev:         previous orientation estimate - assume given
        delta_phi_left:     left wheel rotation (rad)
        delta_phi_right:    right wheel rotation (rad)

    Return:
        x_curr:                  estimated x coordinate
        y_curr:                  estimated y coordinate
        theta_curr:              estimated heading
    """

    # These are random values, replace with your own
    delta_left = R * delta_phi_left
    delta_right = R * delta_phi_right
    delta_center_of_mass = (delta_left + delta_right)/2
    x_curr = x_prev + delta_center_of_mass*np.cos(theta_prev) # The reason we use theta_prev here instead of theta_curr is because we're using the change in x
                                                              # resulting from the theta value of the previous x position.
    y_curr = y_prev + delta_center_of_mass*np.sin(theta_prev)

    delta_theta = (delta_right - delta_left)/baseline
    theta_curr = theta_prev + delta_theta
    
    return x_curr, y_curr, theta_curr
