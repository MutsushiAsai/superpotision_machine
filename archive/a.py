import numpy as np
import pyaudio
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed, wait
from dataclasses import dataclass
from random import random
from scipy import signal

SAMPLE_RATE = 44100


CHUNK = 1024

t = np.linspace(0, 2*np.pi, SAMPLE_RATE, endpoint=False)

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

def synthesize(angles: np.array, params: PolygonalSynthesizeParams) -> tuple[np.ndarray, np.ndarray]:
    in1 = angles * params.frequency
    roll = angles * params.roll
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

streams = {}
pa = pyaudio.PyAudio()
def init_thread():
    key = threading.get_ident()
    streams[key] = pa.open(
        output=True,
        channels=2,
        rate=SAMPLE_RATE,
        format=pyaudio.paFloat32,
        output_device_index=1)

def play(l_audio, r_audio):
    stereo = np.zeros([len(l_audio), 2])
    stereo[:, 0] = l_audio
    stereo[:, 1] = r_audio

    key = threading.get_ident()
    stream = streams[key]
    stream.write(stereo, SAMPLE_RATE)

def main():
    times = [
        (CHUNK*i, min(CHUNK*(i+1), SAMPLE_RATE))
        for i in range(int(SAMPLE_RATE/CHUNK)+1)
    ]

    params = PolygonalSynthesizeParams(
        frequency = random()*600+200, # in1: 0 ~ 2000
        roll = random()*1000 - 500, # -1000 ~ 1000
        n = 2.1 + random() * 47.9,  # 2.1 ~ 100
        teeth = random()*5, # 0.0 ~ 1.0
        fold_ratio = random()*5, # 
        fm_ratio = random()*50, # 
        fm_modulation = 0, #
    )
    with ThreadPoolExecutor(max_workers=2, initializer=init_thread) as pool:
        features = []
        while True:
            for (start_t, end_t) in times:
                features.append(pool.submit(synthesize, t[start_t:end_t], params))

            for feature in as_completed(features):
                rets = feature.result()
                pool.submit(play, *rets)

main()

#pa = pyaudio.PyAudio()
#for x in range(pa.get_device_count()):
#    d = pa.get_device_info_by_index(x)
#    print(x, { 'name' : d['name'] , 'type' : 'output' if d['maxInputChannels'] == 0 else 'input' })
#stream1 = pa.open(
#    output=True,
#    channels=2,
#    rate=SAMPLE_RATE,
#    format=pyaudio.paFloat32,
#    output_device_index=1)
#
#stream2 = pa.open(
#    output=True,
#    channels=1,
#    rate=SAMPLE_RATE,
#    format=pyaudio.paFloat32,
#    output_device_index=1)
#
#t = np.linspace(0, 2*np.pi, SAMPLE_RATE, endpoint=False)
#sina1 = np.sin(t*441).astype(np.float32)
#sina2 = np.sin(t*600).astype(np.float32)
#audio_stereo1 = np.zeros((SAMPLE_RATE, 2), dtype=np.float32)
#audio_stereo1[:, 0] = sina1
#audio_stereo1[:, 1] = sina1
##audio_stereo2 = np.zeros((SAMPLE_RATE, 2), dtype=np.float32)
##audio_stereo2[:, 0] = sina1
##audio_stereo2[:, 1] = sina2
#print(type(audio_streo1), audio_stereo1)
#audio_stereo1 = np.array((sina1, sina1)).T
#audio_stereo2 = np.array((sina2, sina2)).T
#
#print('----------------')
#print(type(audio_streo1), audio_stereo1)
#print(audio_stereo1)
#
#while True:
#    with ThreadPoolExecutor(max_workers=4) as pool:
#        f1 = pool.submit(stream1.write, audio_stereo1, 44100)
#        f2 = pool.submit(stream2.write, audio_stereo2, 44100)
#
#    #wait([f1, f2])
