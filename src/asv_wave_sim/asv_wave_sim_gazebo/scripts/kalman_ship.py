import numpy as np
from filterpy.kalman import KalmanFilter
import filterpy as fp
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation

A1=1
A2 = 0.5
A3 = 1

dt = 0.25

freq1=1
freq2 = 0.3
freq3 = 0.5

drone_al = 10

class Kalman_est():
    def __init__(self):
        self.dt = dt
        self.A = np.array([[1, self.dt, 0, 0, 0], [0, 1, 0, 0, 0,], [0, 0, 1, self.dt, 1/2*self.dt**2], [0, 0, 0, 1, self.dt], [0, 0, 0, 0, 1]])
        self.B = np.array([[1/2*self.dt**2], [self.dt], [0], [0], [0]])
        self.C = np.array([[1, 0, -1, 0, 0], [1, 0, 0, 0, 0]])
        self.D = 0
        self.f = KalmanFilter (dim_x=5, dim_z=2)
        self.f.F = self.A #A matrix
        self.f.H = self.C #C matrix
        #print(self.f.P)
        #self.f.P = np.array([[1., 1., 0., 0., 0.],[1., 1., 0., 0., 0.], [0.,0.,1.,1.,1.],[0.,0.,1.,1.,1.],[0.,0.,1.,1.,1.]])
        #print(self.f.P)
        #self.f.P *= 1000. #Initial covariance
        self.f.B = self.B
        self.f.R = np.array([[0.1, 0],[0,0.1]])
        #self.f.Q = np.array([[self.dt**2, self.dt, 0, 0, 0],[self.dt, self.dt**2, 0, 0, 0], [0, 0, self.dt**4/4, self.dt**3/2, self.dt**2/2], [0, 0, self.dt**3/2, self.dt**2, self.dt], [0, 0, self.dt**2/2, self.dt, 1]])
        self.f.Q = np.array([[1000, 0, 0, 0, 0],[0, 1000, 0, 0, 0], [0, 0, 1000, 0, 0], [0, 0, 0, 3000, 0], [0, 0, 0, 0, 9000]])
        self.u = 0

        

    def init_guess(self, drone_alt):
        self.f.x = np.array([drone_alt, 0., 0, 0, 0])   #Initial guess of states

    def update_control_sig(self,acc):
        self.u = np.array([acc])

    def update_predict(self, dist, drone_alt):
        self.f.predict(u=self.u)
        self.f.update([dist,drone_alt])
        return self.f.x


        


def dist_ship_drone(t): 
    #Calculates ship height, unit here is whatever unit is desired for the filter

    return drone_al-(A1*math.sin(freq1*t) + A2*math.sin(freq2*t) + A3*math.sin(freq3*t))

def ship_vel(t):
    return A1*math.cos(freq1*t)*freq1 + A2*math.cos(freq2*t)*freq2 + A3*math.cos(freq3*t)*freq3


    


if __name__ == '__main__':
    

    f_x = open("log_x.txt", 'a')
    t=0
    kf = Kalman_est()
    kf.init_guess(drone_al)

    z_vec = [dist_ship_drone(i*dt) for i in range(100)]

    x_vec_4 = []
    x_vec_4_gt = []
    t_vec = []
    

    for z in z_vec:
        kf.update_control_sig(0)
        x = kf.update_predict(z, drone_al)
        x_vec_4.append(x[3])
        x_vec_4_gt.append(ship_vel(t))
        t_vec.append(t)
        t=t + dt
        f_x.write(str(x[3]))
        f_x.write(',')
        #print(x[2])
    plt.plot(t_vec, x_vec_4)
    plt.plot(t_vec, x_vec_4_gt)
    plt.show()

    
    

    

