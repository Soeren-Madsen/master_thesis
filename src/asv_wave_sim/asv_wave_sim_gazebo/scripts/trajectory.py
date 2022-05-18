#!/usr/bin/env python3

import math
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import fsolve, bisect
import time


class Trajec():
    def __init__(self, alt):
        #DRONE PARAM, NEEDS TO BE DETERMINED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        self.zrmax_ddot = 1
        self.zrmax_dot = 1
        
        t0 = 0
        self.freq1 = 0.2*2*math.pi
        self.freq2 = 0.12*2*math.pi
        self.freq3 = 0.13*2*math.pi
        z_t0 = alt #Start height of drone
        z_dot_t0 = 0 #Start velocity drone

        self.zd_hat = self.zd(t0) #Platform start pos
        self.zd_dot_hat = self.zd_dot(t0) #Platform start vel

        self.za = z_t0-self.zd_hat
        self.za_dot =z_dot_t0 - self.zd_dot_hat
        self.za_tau = 1

        self.landed = False

    def solve_numeric(self,t, dt):
        if not self.landed:
            eqn = lambda tau : self.za_dot + self.zrmax_ddot*(tau-t)-self.zd_dot(tau)+self.zd_dot(t)
            tf = fsolve(eqn, t) #Solving equation numerical
            #tf = bisect(eqn, t, t+50)
            
            zd_tf = self.zd(tf)
            zd_t = self.zd(t)
            zd_d_tf = self.zd_dot(tf)

            za_tau = self.za + self.za_dot*(tf-t)+1/2*self.zrmax_ddot*(tf-t)**2-zd_tf+zd_t+self.zd_dot(t)*(tf-t)
            za_dot_tau = self.za_dot+self.zrmax_ddot*(tf-t)-zd_d_tf+self.zd_dot(t)

            zd_dd = self.zd_ddot(t)

            if za_tau <= 0.02:
                za_ddot=self.zrmax_ddot-zd_dd
            else:
                if self.za_dot <= -self.zrmax_dot-self.zd_dot(t):
                    za_ddot = -zd_dd
                else:
                    za_ddot = -self.zrmax_ddot - zd_dd
            self.za_dot = self.za_dot + za_ddot*dt
            self.za = self.za + self.za_dot*dt+1/2*za_ddot*dt**2
            if self.za <= 0.02:
                self.landed = True
            zr = self.za + self.zd(t)
            zr_d = self.za_dot + self.zd_dot(t)

        else:
            zr = self.zd(t)
            zr_d = self.zd_dot(t)

        return zr, zr_d


    def zd(self,t):
        return 0.1*math.sin(self.freq1*t+math.radians(70)) + 0.08*math.sin(self.freq2*t+math.radians(30)) + 0.05*math.sin(self.freq3*t) + 1.78

    def zd_dot(self,t):
        return 0.1*math.cos(self.freq1*t+math.radians(70))*self.freq1 + 0.08*math.cos(self.freq2*t+math.radians(30))*self.freq2 + 0.05*math.cos(self.freq3)*self.freq3

    def zd_ddot(self,t):
        return -0.1*math.sin(self.freq1*t+math.radians(70))*self.freq1**2 - 0.08*math.sin(self.freq2*t+math.radians(30))*self.freq2**2 -0.05*math.sin(self.freq3*t)*self.freq3**2
    





if __name__ == '__main__':
    traj = Trajec(6)
    t = np.linspace(0, 5, 50, endpoint=False)
    point_z = []
    point_zd = []
    point_zdd = []
    zref = []
    zd_ref = []
    for ti in t:
        start = time.time()
        zr, zr_d = traj.solve_numeric(ti, 0.1)
        end = time.time()
        print(end-start)
        zref.append(zr)
        zd_ref.append(zr_d)
        point_z.append(traj.zd(ti))
        point_zd.append(traj.zd_dot(ti))
        point_zdd.append(traj.zd_ddot(ti))
    plt.plot(t,point_z)
    plt.plot(t,zref)
    #plt.plot(t,zd_ref)
    #plt.plot(t,point_zd)
    #plt.plot(t,point_zdd)
    plt.show()