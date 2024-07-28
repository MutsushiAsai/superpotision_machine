import numpy as np
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from random import random


def f(v, cb):
    time.sleep(5*random())
    cb(v)
    return v

vals = np.arange(4)


def callback(result):
    print("DONE:", result)

with ThreadPoolExecutor(max_workers=4) as pool:
    features = []
    while True:
        #
        # do heavy process
        #
        for feature in as_completed(features):
            feature.result()
            pass

        features = [pool.submit(f, v, callback) for v in vals]



