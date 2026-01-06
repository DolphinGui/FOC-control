from scipy import signal
from scipy.fft import fft, fftfreq, fftshift
import matplotlib.pyplot as plt
import numpy as np
from math import pi, exp
import re

samples = 100000
sample_length = 10 * pi / 100
sample_period = sample_length / samples
freq = 1 / sample_period
x = np.linspace(0, sample_length, samples)


def first_order_lp(y):
    taps = signal.firwin2(
        4,
        [0.0, 1 / 100, 1 / 50, freq / 2],
        [1.0, 1.0, 0.4, 0.0],
        fs=freq,
    )
    return signal.convolve(taps, y, mode="valid")


def heterodyne(y):
    bp = signal.butter(1, [900, 1100], btype="bandpass", output="sos", fs=freq)
    lbp = signal.butter(1, 110, btype="lowpass", output="sos", fs=freq)
    lo = np.sin(1000 * x * 2 * pi)
    return signal.sosfilt(lbp, lo * signal.sosfilt(bp, y))


def undersample_het(y):
    bp = signal.butter(1, [900, 1100], btype="bandpass", output="sos", fs=freq)
    mixed = signal.sosfilt(bp, y) * np.sin(1000 * x * 2 * pi)
    return np.repeat(mixed[::8], 8)


p = samples // 2


def fourier(x):
    return np.abs(fft(x)[:p])




def do_plot(inject, name):
    y_m = np.sin(x * 100 * 2 * pi)
    y = y_m * inject
    xf = fftfreq(samples, sample_period)[:p]
    y2 = first_order_lp(y)
    y_h = heterodyne(y)
    y_u = undersample_het(y)
    plt.clf()
    plt.xscale("log")
    plt.plot(xf[10:80000], fourier(y_m)[10:80000], label="Signal")
    plt.plot(xf[10:80000], fourier(y)[10:80000], label="Input")
    plt.plot(xf[10:80000], fourier(y2)[10:80000], label="Lowpass")
    plt.plot(xf[10:80000], fourier(y_h)[10:80000], label="Heterodyne")
    plt.plot(xf[10:80000], fourier(y_u)[10:80000], label="Undersampling")
    plt.plot(xf[10:80000], fourier(inject)[10:80000], label="Carrier")
    plt.title(name)
    plt.legend()
    pat = re.compile(r"\s")
    filename = pat.sub("_", name.lower()) + ".jpg"
    plt.savefig(fname="out/" + filename)


do_plot(np.sin(1000 * x * 2 * pi), "Sinusoidal Injection")
do_plot(signal.square(1000 * x * 2 * pi, duty=0.5), "Square Wave Injection")
do_plot(signal.square(1000 * x * 2 * pi, duty=0.1), "10% Pulse Injection")

