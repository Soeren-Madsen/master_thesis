from scipy import zeros, signal, random
import matplotlib.pyplot as plt
import numpy as np
import csv

def filter_sbs():

    T = 60.5         # value taken in seconds
    n = int(60.5 * 12.4) # indicates total samples
    #t = np.linspace(0, T, 1200, endpoint=False)

    #data = np.sin(1.2*2*np.pi*t) + 1.5*np.cos(9*2*np.pi*t) + 0.5*np.sin(12.0*2*np.pi*t)
    gt = []
    data = []
    ship = []
    t = []
    with open('ori_motion.txt') as f:
        csvreader = csv.reader(f)
        for row in csvreader:
            #gt.append(float(row[1]))
            #data.append(-float(row[3])-0.015)
            gt.append(float(row[4])) #roll
            data.append(float(row[0])) #roll
            t.append(float(row[2]))

    for i in range(len(data)):
        if data[i] <0:
            data[i] = data[i]+2*3.1415
        data[i] = data[i]-3.1

    data = np.array(data)
    print(len(data))
    delete = []
    for i in range(110):
        delete.append(i)
    for i in range(650):
        delete.append(1299-i)
    data = np.delete(data, delete)
    gt = np.delete(gt, delete)
    t = np.delete(t, delete)
    t = t-t[0]
    print(len(data))
    b,a = signal.butter(2, 0.35, fs=10)
    #b = signal.firwin(25, 0.01)
    #a = 1
    z = signal.lfilter_zi(b, a)*0
    result = zeros(data.size)
    print(len(result))
    for i, x in enumerate(data):
        result[i], z = signal.lfilter(b, a, [x], zi=z)
    return result,gt,data,t

if __name__ == '__main__':

    result,gt,data,t = filter_sbs()
    plt.plot(t,data, label="Raw data")
    plt.plot(t,result, label = "Filtered roll est")
    plt.plot(t,gt, label = "Ground truth")
    plt.title("Filtered and raw roll estimation with ground truth")
    plt.xlabel("Time [s]")
    plt.ylabel("Angle [Radians]")
    plt.legend(loc="upper right")
    
    #plt.plot(ship)
    plt.show()