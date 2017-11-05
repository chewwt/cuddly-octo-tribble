#!/usr/bin/env python
from darkflow.net.build import TFNet
import imp
import os

DF_PATH = os.path.dirname(imp.find_module('darkflow')[1])

class Net():
    def __init__(self):
        cur_dir = os.getcwd()
        os.chdir(DF_PATH)
        options = {"model": "cfg/yolo-buoy.cfg", "load": 11000, "threshold": 0.3, "gpu":1.0}
        self.tfnet = TFNet(options)

    def process(self, img):
        return self.tfnet.return_predict(img)