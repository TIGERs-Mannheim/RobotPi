#!/usr/bin/env python3
"""
Converts between yuv and regular image formats.

YUV image format specification. All values in little-endian byte order.
uint64: Microsecond-Timestamp
uint16: Y-Width
uint16: Y-Height
uint8[width * height]: Full resolution Y data
uint8[width/2 * height/2]: Quarter resolution U data
uint8[width/2 * height/2]: Quarter resolution V data
"""
import os
import struct
import sys

import cv2
import numpy as np


def yuv_to_img(path):
    with open(path, "rb") as file:
        _timestamp, width, height = struct.unpack("<QHH", file.read(12))
        y = np.frombuffer(file.read(width*height), '<u1').reshape((height, width))
        quarter_size = (width//2) * (height//2)
        u = np.frombuffer(file.read(quarter_size), '<u1').reshape((height//2, width//2))
        v = np.frombuffer(file.read(quarter_size), '<u1').reshape((height//2, width//2))

    u = np.repeat(np.repeat(u, 2, axis=0), 2, axis=1)
    v = np.repeat(np.repeat(v, 2, axis=0), 2, axis=1)
    yuv = np.stack((y, u, v), axis=-1)

    img = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR)
    cv2.imwrite(path[:-4] + ".png", img)


def img_to_yuv(path):
    img = cv2.imread(path)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
    y = np.array(img, dtype='<u1')[:, :, 0]
    uv = np.array(cv2.resize(img, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA), dtype='<u1')[:, :, 1:]

    with open(path[:-4] + ".yuv", "wb") as file:
        file.write(struct.pack("<QHH", int(os.path.getctime(path) * 1e6), img.shape[1], img.shape[0]))
        file.write(y.tobytes())
        file.write(uv[:, :, 0].tobytes())
        file.write(uv[:, :, 1].tobytes())


if __name__ == '__main__':
    if sys.argv[1].endswith('.yuv'):
        yuv_to_img(sys.argv[1])
    else:
        img_to_yuv(sys.argv[1])
