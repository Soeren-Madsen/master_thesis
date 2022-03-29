import numpy as np
from filterpy.kalman import KalmanFilter
import filterpy as fp
import math
import matplotlib.pyplot as plt

A1=1
A2 = 0.5
A3 = 1

freq1=1
freq2 = 0.3
freq3 = 0.5

drone_alt = 10.

def dist_ship_drone(t): 
    #Calculates ship height, unit here is whatever unit is desired for the filter

    return drone_alt-(A1*math.sin(freq1*t) + A2*math.sin(freq2*t) + A3*math.sin(freq3*t))

def ship_vel(t):
    return A1*math.cos(freq1*t)*freq1 + A2*math.cos(freq2*t)*freq2 + A3*math.cos(freq3*t)*freq3

def setup(dt):
    A = np.array([[1, dt, 0, 0, 0], [0, 1, 0, 0, 0,], [0, 0, 1, dt, 1/2*dt**2], [0, 0, 0, 1, dt], [0, 0, 0, 0, 1]])
    B = np.array([[1/2*dt**2], [dt], [0], [0], [0]])
    C = np.array([[1, 0, -1, 0, 0], [1, 0, 0, 0, 0]])
    D = 0
    return A, B, C, D

if __name__ == '__main__':
    t=0
    dt = 0.2
    u = np.array([0.])
    f = KalmanFilter (dim_x=5, dim_z=2)

    f.x = np.array([drone_alt,0., 0, A1+A2+A3, 0])   #Initial guess of states
    A, B, C, D = setup(dt)
    f.F = A #A matrix
    f.H = C #C matrix
    f.P *= 1000. #Initial covariance
    f.B = B
    f.R = np.array([[1, 0],[0,1]])
    f.Q = np.array([[1000, 0, 0, 0, 0],[0, 1000, 0, 0, 0], [0, 0, 1000, 0, 0], [0, 0, 0, 3000, 0], [0, 0, 0, 0, 9000]])

    z_vec = [dist_ship_drone(i*dt) for i in range(100)]

    x_vec_4 = []
    x_vec_4_gt = []
    t_vec = []
    for z in z_vec:
        f.predict(u=u)
        f.update([z,drone_alt])
        x_vec_4.append(f.x[3])
        x_vec_4_gt.append(ship_vel(t))
        t_vec.append(t)
        t=t + dt

    
    plt.plot(t_vec, x_vec_4)
    plt.plot(t_vec, x_vec_4_gt)
    plt.show()

    

