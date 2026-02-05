from scipy import signal
from argparse import ArgumentParser, BooleanOptionalAction
import matplotlib.pyplot as plt
from term_image.image import AutoImage, KittyImage, ITerm2Image
import numpy as np
from PIL import Image
import sys
import io

argparser = ArgumentParser(
    prog="filt_gen",
    usage="filt_gen [lowpass, highpass, bandpass, bandstop]",
    description="Generate coefficients for filters based on specifications",
)

argparser.add_argument(
    "type",
    choices=["lowpass", "lp", "highpass", "hp", "bandpass", "bp", "bandstop", "bs"],
    help="Type of filter",
)

argparser.add_argument(
    "fs",
    type=float,
    help="Frequency filter is sampled at in Hz",
)

argparser.add_argument(
    "fc",
    type=float,
    help="Corner frequency for highpass and lowpass filters, lower frequency for band filters",
)

argparser.add_argument(
    "fc2", nargs="?", type=float, help="Higher frequency for band filters", default=0
)

argparser.add_argument(
    "--order",
    type=int,
    help="Filter order",
    default=1,
)

argparser.add_argument(
    "--gui",
    type=bool,
    help="Force GUI output for response graph",
    action=BooleanOptionalAction,
)
fs = 10
print(signal.firwin2(20, [0, fs / 2], [1.0, 0], fs = fs))

exit(0)

def main():
    args = argparser.parse_args()
    if (
        args.type == "lowpass"
        or args.type == "lp"
        or args.type == "highpass"
        or args.type == "hp"
    ):
        a = signal.butter(
            args.order, args.fc, btype=args.type, output="sos", fs=args.fs
        )
    elif (
        args.type == "bandpass"
        or args.type == "bp"
        or args.type == "bandstop"
        or args.type == "bs"
    ):
        if args.fc2 == 0:
            print("Second corner frequency must be specified", file=sys.stderr)
            exit(1)
        a = signal.butter(
            args.order, [args.fc, args.fc2], btype=args.type, output="sos", fs=args.fs
        )
    print(a)
    w, h = signal.freqz_sos(a, fs=args.fs)
    plt.semilogx(w, 20 * np.log10(abs(h)))
    plt.title("Frequency Response")
    plt.xlabel("Frequency [rad/s]")
    plt.ylabel("Amplitude [dB]")
    plt.margins(0, 0.1)
    plt.grid(which="both", axis="both")
    plt.axvline(args.fc, color="green")  # cutoff frequency
    if (
        args.type == "bandpass"
        or args.type == "bp"
        or args.type == "bandstop"
        or args.type == "bs"
    ):
        plt.axvline(args.fc2, color="green")  # cutoff frequency
    term_supported = KittyImage.is_supported() or ITerm2Image.is_supported()
    if args.gui or not term_supported:
        plt.show()
    else:
        buf = io.BytesIO()
        plt.savefig(buf)
        buf.seek(0)
        img = AutoImage(Image.open(buf))
        img.draw()


# def fig2img(fig):
#     buf = io.BytesIO()
#     fig.savefig(buf)
#     buf.seek(0)
#     img = Image.open(buf)
#     return img

if __name__ == "__main__":
    main()
