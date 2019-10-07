import numpy as np


r2d = 180.0 / np.pi
d2r = np.pi / 180.0
inv_360 = 1.0 / 360.0
inv_180 = 1.0 / 180.0
inv_pi = 1.0 / np.pi
inv_2pi = 0.5 / np.pi

def deg_wrap_180( angle ):
    """wrap an angle in degrees, -180 <= theta < 180"""
    angle -= 360.0 * np.floor((angle + 180.) * inv_360)
    return angle
#
def deg_wrap_360( angle ):
    """wrap an angle in degrees, 0 <= theta < 360"""
    angle -= 360.0 * np.floor(angle * inv_360)
    return angle
#

# def rad_wrap_pi( angle ):
def wrap( angle ):
    # """wrap an angle in rads, -pi <= theta < pi"""
    angle -= 2*np.pi * np.floor((angle + np.pi) * inv_2pi)
    return angle
#
def rad_wrap_2pi( angle ):
    """wrap an angle in rads, 0 <= theta < 2*pi"""
    angle -= 2*np.pi * np.floor(angle * inv_2pi)
    return angle
#