#!/usr/bin/env python3

import numpy
from ultralytics import YOLO

model = YOLO('yolov8x-seg.pt')

print(model.names)