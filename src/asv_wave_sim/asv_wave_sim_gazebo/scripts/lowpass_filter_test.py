from scipy import zeros, signal, random
import matplotlib.pyplot as plt
import numpy as np
import csv

def filter_sbs():

    T = 5.0         # value taken in seconds
    n = int(T * 10) # indicates total samples
    t = np.linspace(0, T, n, endpoint=False)

    #data = np.sin(1.2*2*np.pi*t) + 1.5*np.cos(9*2*np.pi*t) + 0.5*np.sin(12.0*2*np.pi*t)
    gt = []
    data = []
    ship = []
    with open('log_pitch.txt') as f:
        csvreader = csv.reader(f)
        for row in csvreader:
            gt.append(float(row[1]))
            data.append(-float(row[3])-0.015)
            ship.append(float(row[7])/20)

    data = np.array(data)
    #b,a = signal.butter(2, 0.1, fs=10)
    b = signal.firwin(15, 0.1)
    z = signal.lfilter_zi(b, 1)*0
    result = zeros(data.size)
    for i, x in enumerate(data):
        result[i], z = signal.lfilter(b, 1, [x], zi=z)
    return result,gt,data,ship

if __name__ == '__main__':
    result,gt,data, ship = filter_sbs()
    plt.plot(result)
    plt.plot(gt)
    #plt.plot(ship)
    plt.show()