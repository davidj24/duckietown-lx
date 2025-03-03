from typing import Tuple

import numpy as np


def PIDController(
        v_0: float,
        theta_ref: float,
        theta_hat: float,
        prev_e: float,
        prev_int: float,
        delta_t: float
) -> Tuple[float, float, float, float]:
    """
    PID performing heading control.
    Args:
        v_0:        linear Duckiebot speed (given).
        theta_ref:  reference heading pose.
        theta_hat:  the current estiamted theta.
        prev_e:     tracking error at previous iteration.
        prev_int:   previous integral error term.
        delta_t:    time interval since last call.
    Returns:
        v_0:     linear velocity of the Duckiebot
        omega:   angular velocity of the Duckiebot
        e:       current tracking error (automatically becomes prev_e at next iteration).
        e_int:   current integral error (automatically becomes prev_int at next iteration).
    """

    e = theta_ref - theta_hat
    e_deriv = (e - prev_e)/delta_t
    e_int = prev_int + e * delta_t
    

    k_p = 5
    k_i = 1
    k_d = 1

    # The reason that omega isn't just the derivative of the error is because omega is the derivative of the heading whereas the error isn't
    # The actual heading of the rovot but instead it's heading's relationship to the correct heading.
    omega = (k_p * e) + (k_i * e_int) + (k_d * e_deriv)

    # In this case our u_t is equal to our omega because u_t is normally a vector including omega as one of its entries but since
    # we assume velocity to be constant, we are simplyifying this function and simply calculating a scalar omega and setting it as our u_t.
    # In reality, we are returning the u_t vector and a few extra values but we're just sending out the entries independently rather than in a vector.


    # Hint: print for debugging
    # print(f"\n\nDelta time : {delta_t} \nE : {np.rad2deg(e)} \nE int : {e_int} \nPrev e : {prev_e} \nU : {u} \nTheta hat: {np.rad2deg(theta_hat)} \n")
    # ---
    return v_0, omega, e, e_int
