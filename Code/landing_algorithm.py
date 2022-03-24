import numpy as np
from filterpy import kalman
import filterpy as fp
import math

A1=10


freq1=1


drone_alt = 100.

'''
NEXT TIME: Højde værdien kan ende i negativ hvilket ikke giver mening
'''

def calc_ship_alt(t): 
    #Calculates ship height, unit here is whatever unit is desired for the filter


    return drone_alt-(A1*math.sin(freq1*t))


def calc_ground_truth(t):
    gt = np.full([5],None)
    gt[0] = drone_alt
    gt[1] = a*t
    gt[2] = A1*math.sin(freq1*t)
    gt[3] = A1*math.cos(freq1*t)
    gt[4] = freq1

    return gt

def fx(x, dt, **arg):
    #-------------------------------------------------------NEW--------------------------------------------------
    a=arg.get('acc')
    x_new = np.full([5],None)
    
    #Own States:
    #x_new[0] = x[0]+x[1]*dt
    #x_new[1] = x[1]+a*dt 
    #x_new[2] = x[2]+x[4]*x[3]*dt
    #x_new[3] = x[3]-x[4]*x[2]*dt
    #x_new[4] = x[4]

    #Papers states:
    x_new[0] = x[0] + 0.5*x[1]*dt
    x_new[1] = x[1]
    x_new[2] = x[2]*math.cos(x[4]*dt)+x[3]*math.sin(x[4]*dt)
    x_new[3] = -math.sin(x[4]*dt)*x[3]+math.cos(x[4]*dt)*x[3]
    x_new[4] = x[4]
    return x_new

    
    
def hx(x):
    # measurement function - convert state into a measurement
    # where measurements are [x_pos, y_pos]
    #return np.array([x[0], x[2]])
    y = x[0]-x[2]
    return [y,x[0]]
    

if __name__ == '__main__':
    t=0
    dt = 0.1
    a = 0
    points=fp.kalman.MerweScaledSigmaPoints(5, alpha=0.001, beta=2., kappa=0)
    #print(points)
    kf = kalman.UnscentedKalmanFilter(dim_x=5, dim_z=2, dt=dt, fx=fx, hx=hx, points=points)
    kf.x = np.array([drone_alt, 0., 0., A1, freq1]) # initial state
    kf.R *= 0.1 # 1 standard
    #kf.Q = fp.common.Q_discrete_white_noise(dim=2, dt=dt, var=0.01**2, block_size=3)
    kf.Q *= 0.1
    zs = [calc_ship_alt(i*dt) for i in range(1000)]
    for z in zs:
        t=t+dt
        ship_alt = calc_ship_alt(t)
        #print(ship_alt)
        kf.predict(acc=a)
        kf.update([z,drone_alt])
        gt = calc_ground_truth(t)
        #print("states: ", kf.x, 'log-likelihood', kf.log_likelihood)
        #print(kf.x[4])
        #print("ground truth: {}".format(gt))
        print(kf.x-gt)
