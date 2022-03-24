import numpy as np
from filterpy import kalman
import filterpy as fp
import math

A1=1

freq1=1

drone_alt = 10.

def dist_ship_drone(t): 
    #Calculates ship height, unit here is whatever unit is desired for the filter

    return drone_alt-(A1*math.sin(freq1*t))

def setup(dt):
    A = np.matrix([[1, dt, 0, 0, 0], [0, 1, 0, 0, 0,], [0, 0, 1, dt, 1/2*dt**2], [0, 0, 0, 1, dt]])

    return A

if __name__ == '__main__':
    dt = 0.1
    A = setup(dt)
    print(A)