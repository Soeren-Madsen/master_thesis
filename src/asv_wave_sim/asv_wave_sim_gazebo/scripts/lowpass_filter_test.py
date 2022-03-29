from scipy import zeros, signal, random
import matplotlib.pyplot as plt
import numpy as np

def filter_sbs():

    T = 5.0         # value taken in seconds
    n = int(T * 100) # indicates total samples
    t = np.linspace(0, T, n, endpoint=False)

    data = np.sin(1.2*2*np.pi*t) + 1.5*np.cos(9*2*np.pi*t) + 0.5*np.sin(12.0*2*np.pi*t)
    #data = random.random(2000)
    b = signal.firwin(25, 0.02)
    z = signal.lfilter_zi(b, 1)*0
    print(z)
    result = zeros(data.size)
    for i, x in enumerate(data):
        result[i], z = signal.lfilter(b, 1, [x], zi=z)
        print([x])
    return result,data

if __name__ == '__main__':
    result,data = filter_sbs()
    plt.plot(result)
    plt.plot(data)
    plt.show()