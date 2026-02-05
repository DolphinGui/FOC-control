from scipy import signal
from scipy.fft import fft, fftfreq, fftshift
import matplotlib.pyplot as plt
import numpy as np
from numpy import cos, sin
from math import pi, exp, sqrt
import re

# all frequencies are in hz / rotations per second
fpwm = 100_000
ffoc = 10_000
fhfi = 1_000
frotor = 100

t = 1 / frotor

# n refers to sample
n_samples = 10
# time axis of the motor
tm = np.linspace(0, t, n_samples * fpwm)
# time axis of filter
tf = np.linspace(0, t, n_samples * ffoc)


# expects a signal of length tf
def sin_voltage(d):
    # for sake of analysis we assume we already know the angle. we are analyzing
    # the filter's ability to distinguish motor angle HFI carrier frequency
    q = np.zeros(np.shape(d))
    dq = np.stack([d, q])
    clarke = sqrt(2 / 3) * np.matrix(
        [[1, 0], [-0.5, sqrt(3) / 2], [-0.5, -sqrt(3) / 2]]
    )
    theta = tf * 2 * pi * frotor
    c = cos(theta)
    s = sin(theta)

    park = np.stack([[c, s], [-s, c]])  # shape: (100, 2, 2)
    aby = np.einsum("ijn,in->jn", park, dq)
    abc = np.einsum("ij,jn->in", clarke, aby)
    ratio = fpwm / ffoc
    abc = np.repeat(abc, ratio, axis=1)
    print(f"abc shape: {np.shape(abc)}")
    print(f"tm shape: {np.shape(tm)}")
    print(f"ratio: {ratio}")
    time = tm * fpwm / frotor * 2 * pi
    v_a = signal.square(time, duty=(abc[0] + 1) / 2) * 2 - 1
    v_b = signal.square(time, duty=(abc[1] + 1) / 2) * 2 - 1
    v_c = signal.square(time, duty=(abc[2] + 1) / 2) * 2 - 1
    return np.array([v_a, v_b, v_c])


def motor(v):
    # based on https://www.cubemars.com/product/ak60-39-v3-0-kv80-robotic-actuator.html
    r = 600e-3
    l = 670e-6
    # a made up number that is hopefully good enough, l_d / l_q
    saliency = 1.2
    def i_motor(t, v, theta):

    pass


def do_plot(inject, name):
    y = sin_voltage(inject)
    print(f"t shape: {np.shape(tm)}")
    print(f"y shape: {np.shape(y)}")
    plt.plot(tm, y[0], label="a")
    plt.plot(tm, y[1], label="b")
    plt.plot(tm, y[2], label="c")
    plt.title(name)
    # plt.legend()
    plt.show()
    # pat = re.compile(r"\s")
    # filename = pat.sub("_", name.lower()) + ".jpg"
    # plt.savefig(fname="out/" + filename)


# do_plot(np.ones(n_samples), "one")
do_plot(signal.square(tf * 2 * pi * 1), "square")
