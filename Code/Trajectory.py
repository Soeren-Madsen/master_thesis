import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.optimize import fsolve
import time

update_rate_hz = 20

def zd(t,freq,A):
    return A*math.sin(freq*t)

def zd_dot(t,freq,A):
    return A*math.cos(freq*t)*freq

def zd_ddot(t,freq,A):
    return -A*math.sin(freq*t)*freq**2

def calc_traject(t0,z_t0,z_dot_t0):
    #Parameters for the wave prediction. Needs to be estimated for real implementation
    A1=0.3
    A2=0.2
    A3=0.5
    freq1=1
    freq2=2
    freq3=0.5

    #Parameters
    zrmax_dot = 2 #Max trajectory velocity (Drone velocity can be higher, since the wave velocity can be added)
    zrmax_ddot = 5 #Max trajectory acceleration (Drone acceleration can be higher, since the wave acceleration can be added)
    dt = 1/update_rate_hz #Time step
    time_pred = 10

    #Initial values
    zd_hat = zd(t0,freq1, A1) + zd(t0,freq2, A2) + zd(t0,freq3, A3)
    zd_dot_hat = zd_dot(t0,freq1, A1) + zd_dot(t0,freq2, A2) + zd_dot(t0,freq3, A3)
    za = z_t0 - zd_hat
    za_dot = z_dot_t0 - (zd_dot_hat)


    time = np.linspace(t0,t0+time_pred,update_rate_hz*time_pred+1)
    
    #zr_vec = []
    #zd_vec = []

    for t in time:
        #print(t)
        #Function to calculate at what time we can get to 0 velocity
        zd_d_t = zd_dot(t,freq1,A1) + zd_dot(t,freq2,A2) + zd_dot(t,freq3,A3)
        #print(zd_d_t)
        func = lambda tau: za_dot + zrmax_ddot*(tau-t)-(zd_dot(tau,freq1, A1) + zd_dot(tau,freq2, A2) + zd_dot(tau,freq3, A3)) + zd_d_t
        tau_init_guess = t
        tf = fsolve(func, tau_init_guess) #Time we can get to 0 velocity
        #print(tf)
        zd_tf = zd(tf,freq1,A1) + zd(tf,freq2,A2) + zd(tf,freq3,A3) #Calculate ship pos at tf
        zd_t = zd(t,freq1,A1) + zd(t,freq2,A2) + zd(t,freq3,A3) #Calculate ship pos at current time
        zd_d_tf = zd_dot(tf,freq1,A1) + zd_dot(tf,freq2,A2) + zd_dot(tf,freq3,A3) #Calculate ship velocity at tf
        za_tau = za + za_dot*(tf-t)+1/2*zrmax_ddot*(tf-t)**2-zd_tf+zd_t+zd_d_t*(tf-t) #Calculate different between ref and ship pos at tf
        za_dot_tau = za_dot+ zrmax_ddot*(tf-t)-zd_d_tf+zd_d_t #Calculate difference between ship and drone velocity at tf (Should be 0), not necessary to calculate

        zd_dd = zd_ddot(t,freq1,A1) + zd_ddot(t,freq2,A2) + zd_ddot(t,freq3,A3)

        if za_tau <= 0:
            za_ddot = zrmax_ddot-zd_dd
        elif za_tau > 0:
            if za_dot <= -zrmax_dot - zd_d_t:
                za_ddot = -zd_dd
            else:
                za_ddot = -zrmax_ddot - zd_dd
        
        za_dot = za_dot + za_ddot*dt
        
        za = za + za_dot*dt+1/2*za_ddot*dt**2

        zr = za+zd_t
        #zr_vec.append(zr)
        #zd_vec.append(zd_t)
    #print(zr_vec)
    #plt.plot(time,zr_vec)
    #plt.plot(time,zd_vec)
    #plt.show()
    
        



if __name__ == '__main__':
    #Values for when the trajectory prection should start.
    t0 = 0
    z_t0 = 10
    z_dot_t0 = 0
    start = time.time()
    calc_traject(t0, z_t0, z_dot_t0)
    end = time.time()
    t = end-start
    print("Time for calculation: ", t)