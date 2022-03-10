import numpy as np
from filterpy import kalman
import filterpy as fp
import math

A1=10
A2=20
A3=30

freq1=2
freq2=1
freq3=4

'''
NEXT TIME: Højde værdien kan ende i negativ hvilket ikke giver mening
'''

def calc_ship_alt(t): 
    #Calculates ship height, unit here is whatever unit is desired for the filter


    return 100-(A1*math.sin(freq1*t)+A2*math.sin(freq2*t)+A3*math.sin(freq3*t))


def calc_ground_truth(t):
    gt = np.full([11],None)
    gt[0] = 10
    gt[1] = a*t
    gt[2] = A1*math.sin(freq1*t)
    gt[3] = A1*math.cos(freq1*t)
    gt[4] = freq1
    gt[5] = A2*math.sin(freq2*t)
    gt[6] = A2*math.cos(freq2*t)
    gt[7] = freq2
    gt[8] = A3*math.sin(freq3*t)
    gt[9] = A3*math.cos(freq3*t)
    gt[10] = freq3

    return gt

def fx(x, dt, **arg):
    #-------------------------------------------------------NEW--------------------------------------------------
    a=arg.get('acc')
    x_new = np.full([11],None)
    
    x_new[0] = x[0]+0.5*x[1]*dt+0.25*a*dt**2
    x_new[1] = x[1]+0.5*a*dt #Fejl? burde det være uden at være opløftet til i anden? Orginal er x[1]+0.5*a*dt
    x_new[2] = x[2]*math.cos(x[4]*dt)+x[3]*math.sin(x[4]*dt)
    x_new[3] = -math.sin(x[4]*dt)*x[2]+math.cos(x[4]*dt)*x[3] #Fejl? Burde det første x[3] være et x[2] i stedet? Det er det i de andre gange dette bliver regnet. original er: -math.sin(x[4]*dt)*x[3]+math.cos(x[4]*dt)*x[3]
    x_new[4] = x[4]
    x_new[5] = x[5]*math.cos(x[7]*dt)+x[6]*math.sin(x[7]*dt)
    x_new[6] = -math.sin(x[7]*dt)*x[5]+math.cos(x[7]*dt)*x[6]
    x_new[7] = x[7]
    x_new[8] = x[8]*math.cos(x[10]*dt)+x[9]*math.sin(x[10]*dt)
    x_new[9] = -math.sin(x[10]*dt)*x[8]+math.cos(x[10]*dt)*x[9]
    x_new[10] = x[10]
    return x_new

    
    
def hx(x):
    # measurement function - convert state into a measurement
    # where measurements are [x_pos, y_pos]
    #return np.array([x[0], x[2]])
    y = x[0]-x[2]-x[5]-x[8]
    return [y]
    

if __name__ == '__main__':
    t=0
    dt = 0.1
    a = 0
    points=fp.kalman.MerweScaledSigmaPoints(11, alpha=0.1, beta=2., kappa=-1)
    #print(points)
    kf = kalman.UnscentedKalmanFilter(dim_x=11, dim_z=1, dt=dt, fx=fx, hx=hx, points=points)
    kf.x = np.array([100., 0., 0., 10., 2., 0., 20., 1., 0., 30., 4.]) # initial state
    kf.P *= 0.1
    z_std = 1
    kf.R = z_std**2 # 1 standard
    kf.Q[0:2,0:2]=fp.common.Q_discrete_white_noise(2, dt=dt, var=0.1)
    kf.Q[2:5,2:5]=fp.common.Q_discrete_white_noise(3, dt=dt, var=0.1)
    kf.Q[5:8,5:8]=fp.common.Q_discrete_white_noise(3, dt=dt, var=0.1)
    kf.Q[8:11,8:11]=fp.common.Q_discrete_white_noise(3, dt=dt, var=0.1)
    #kf.Q = fp.common.Q_discrete_white_noise(dim=2, dt=dt, var=0.01**2, block_size=3)
    zs = [calc_ship_alt(i*dt) for i in range(1000)]
    for z in zs:
        t=t+dt
        ship_alt = calc_ship_alt(t)
        #print(ship_alt)
        kf.predict(acc=a)
        kf.update(z)
        gt = calc_ground_truth(t)
        #print("states: ", kf.x, 'log-likelihood', kf.log_likelihood)
        print(calc_ship_alt(t)-(kf.x[0]-kf.x[2]-kf.x[5]-kf.x[8]))
        #print(kf.x[2]+kf.x[5] + kf.x[8])
        #print("ground truth: {}".format(gt))
        #print(kf.x-gt)
        #print(z)
