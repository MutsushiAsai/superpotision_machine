import numpy as np
import numpy.typing as npt
#import matplotlib as mpl; mpl.use('tkagg'); mpl.rcParams['toolbar'] = 'None' # for windows 
import matplotlib as mpl; mpl.use('qtagg'); mpl.rcParams['toolbar'] = 'None' # for linux
import matplotlib.pyplot as plt
import matplotlib.style as mplstyle; mplstyle.use('fast')
import pyaudio
import serial
import struct
import sys

from collections import namedtuple
from concurrent.futures import as_completed, ThreadPoolExecutor
from dataclasses import dataclass
from scipy import signal

device_count = 4

DIVIDER = 10
SAMPLE_RATE = 44100
FRAME_SIZE = int(SAMPLE_RATE / DIVIDER)

DRAW_POINTS = 18

def fold(v, vmin, vmax):
    range_v = vmax - vmin
    fv = np.mod(v - vmin, 2 * range_v)
    fv = np.where(fv > range_v, 2 * range_v - fv, fv)
    return fv + vmin

def sawtooth(*args, **kwargs):
    return (signal.sawtooth(*args, **kwargs) + 1) * .5


@dataclass
class PolygonalSynthesizeParams:
    frequency: float
    roll: float
    n: float
    teeth: float
    fold_ratio: float
    fm_ratio: float
    fm_modulation: float

def synthesize(t: np.array, params: PolygonalSynthesizeParams) -> tuple[np.ndarray, np.ndarray]:
    in1 = t * params.frequency
    roll = t * params.roll
    n = params.n
    teeth = params.teeth
    fold_ratio = params.fold_ratio
    fm_ratio = params.fm_ratio
    fm_modulation = params.fm_modulation

    theta_zero = np.pi / n
    phi = 2 * np.pi * sawtooth(in1 + fm_modulation * np.sin(fm_ratio*in1))
    rotate = 2 * np.pi * sawtooth(roll)
    T = np.pi * (n - 2)/(2 * n) * teeth
    theta = 2 * theta_zero * np.mod(phi*n / 2*np.pi, 1)
    p = np.cos(theta_zero + T) / np.cos(theta - theta_zero + T)
    out_x = fold(fold_ratio * p * np.cos(phi + rotate), -1, 1)
    out_y = fold(fold_ratio * p * np.sin(phi + rotate), -1, 1)

    return out_x, out_y

@dataclass
class ChangeControlFrame:
    channel: int
    control_number: int
    value: int

ports = [
    "/dev/ttyUSB0",
    "/dev/ttyUSB1",
    "/dev/ttyUSB2",
    "/dev/ttyUSB3",
]

polygonal_synthesize_params = [
    PolygonalSynthesizeParams(
        frequency = 0, # in1: 0 ~ 2000
        roll = 0, # -1000 ~ 1000
        n = 2.1,  # 2.1 ~ 100
        teeth = 0, # 0.0 ~ 1.0
        fold_ratio = 0, # 
        fm_ratio = 0, # 
        fm_modulation = 0, #
    )
    for i in range(device_count)
]

class PolyGonalSynthesisVisualizer:
    def __init__(self, sample_rate: int=44100, synthesizer_count: int = 4, plots_opts: list[dict] = []):
        self.sample_rate = sample_rate
        self._x = np.zeros(sample_rate)
        self._y = np.zeros(sample_rate)

        self._synthesizer_count = synthesizer_count

        fig, axis = plt.subplots(figsize=(7, 7), facecolor="black")
        fig.subplots_adjust(left=0, right=1, top=1, bottom=0)
        fig.canvas.header_visible = False
        fig.canvas.footer_visible = False
        fig.canvas.toolbar_visible = False
        axis.set_facecolor("black")
        axis.get_xaxis().set_ticks([])
        axis.get_yaxis().set_ticks([])
        axis.spines['top'].set_visible(False)
        axis.spines['right'].set_visible(False)
        axis.spines['bottom'].set_visible(False)
        axis.spines['left'].set_visible(False)
        axis.set_xlim(-1, 1)
        axis.set_ylim(-1, 1)
        zero = np.zeros(DRAW_POINTS)

        def on_closed(evt):
            import sys; sys.exit(0)

        fig.canvas.mpl_connect('close_event', on_closed)

        lines = []
        for i in range(synthesizer_count):
            plot_opts = {"linewith": 0.5}
            try:
                plot_opts = plots_opts[i]
            except IndexError:
                pass
            line, = axis.plot(zero, zero, **plot_opts)
            lines.append(line)

        wm = plt.get_current_fig_manager()
        # wm.window.state('zoomed') # for windows
        #wm.full_screen_toggle() # for linux

        self._figure = fig
        self._axis = axis
        self._lines = lines
        self._synthesizer_data = [
            [zero, zero] for _ in range(synthesizer_count)
        ]

        plt.show(block=False)
    
    def set_data(self, index: int, x: np.array, y: np.array) -> None:
        self._synthesizer_data[index] = [x, y]

    def draw(self, j):
        v = DRAW_POINTS 
        x, y = self._synthesizer_data[j]
        self._lines[j].set_data(x[:v], y[:v])
        self._figure.canvas.draw()

    def update(self):
        self._figure.canvas.flush_events()

import time, threading
is_running = False

def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def handle_message(index):
    global is_running
    global ports
    global polygonal_synthesize_params 

    last_connection_status = True
    try:
        port = ports[index]
        params = polygonal_synthesize_params[index]

        def handle_serial(port):
            with serial.Serial(port, 38400) as ser:
                print(f'connection established: "{port}"')

                while True:
                    v = struct.unpack('B', ser.read(1))
                    if len(v) == 0:
                        continue

                    v = int(v[0])
                    if (v & 0xF0) == 0:
                        continue

                    status = v & 0xF0
                    channel = v & 0x0F

                    if status != 0xB0:
                        continue

                    control_number, value = struct.unpack('2B', ser.read(2))
                    cc = ChangeControlFrame(channel, control_number, value)
                    if control_number != 0x07: # 0x07: main volume ( midi )
                        continue
                    value = cc.value
                    if cc.channel == 1:
                        params.frequency = map_range(value, 0, 127, 0, 2000.0)
                    elif cc.channel == 2:
                        params.roll = map_range(value, 0, 127, 0, 200.0)
                    elif cc.channel == 3:
                        params.n = map_range(value, 0, 127, 2.1, 150.0)
                    elif cc.channel == 4:
                        params.teeth =  map_range(value, 0, 127, 0, 0.99)
                    elif cc.channel == 5:
                        params.fold_ratio = map_range(value, 0, 127, 0, 100.0) 
                    elif cc.channel == 6:
                        params.fm_ratio = map_range(value, 0, 127, 0, 10.0)
                    elif cc.channel == 7:
                        params.modulation = map_range(value, 0, 127, 0, 100.0)

        while is_running:
            try:
                handle_serial(port)

                last_connection_status = True
            except serial.serialutil.SerialException as ex:
                if last_connection_status:
                    print(f'connection lost: try to connect "{port}"')
                time.sleep(2)
                last_connection_status = False
            except Exception as ex:
                print(ex)

    except Exception as ex:
        print(ex)

def main():
    global is_running
    viz = PolyGonalSynthesisVisualizer(SAMPLE_RATE, device_count, plots_opts=[
        {"color":"r", "linewidth": .6},
        {"color":"g", "linewidth": .8},
        {"color":"b", "linewidth": .5},
        {"color":"w", "linewidth": .3},
    ])

    is_running = True
    for i in range(device_count):
        t = threading.Thread(target=handle_message, args=[i])
        t.start()

    pa = pyaudio.PyAudio()
    pool = None
    try:
        streams = {}
        def initializer():
            key = threading.get_ident()
            streams[key] = pa.open(
                            output=True,
                            channels=2,
                            rate=SAMPLE_RATE,
                            format=pyaudio.paFloat32,
                            output_device_index=1)

        pool = ThreadPoolExecutor(max_workers=device_count, initializer=initializer)
        def play(x, y):
            stereo = np.zeros((len(x), 2), dtype=np.float32)
            key = threading.get_ident()
            stream = streams[key]
            stereo[:, 0] = x
            stereo[:, 1] = y
            streams[key].write(stereo, FRAME_SIZE)

        time_ranges = [
            (FRAME_SIZE*i, min(FRAME_SIZE*(i+1), SAMPLE_RATE))
            for i in range(int(SAMPLE_RATE/FRAME_SIZE)+1)
        ]
        t = np.linspace(0, 2*np.pi, SAMPLE_RATE, endpoint=False)
 
        def _f(i, t, params):
            ret = synthesize(t, params)
            return i, ret

        while True:
            for st, et in time_ranges:
                features = [
                    pool.submit( _f, i, t[st:et], polygonal_synthesize_params[i])
                    for i in range(device_count)
                ]

                for i in range(device_count):
                    viz.draw(i)

                viz.update()

                for feature in as_completed(features):
                    i, (x, y) = feature.result()
                    viz.set_data(i, x, y)
                    pool.submit(play, x, y)

    except KeyboardInterrupt:
        pass

    if pool:
        pool.shutdown(wait=False, cancel_futures=True)

    is_running = False

if __name__ == '__main__':
    main()
