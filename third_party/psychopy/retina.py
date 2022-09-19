#!/usr/bin/env python
# The contents of this file are in the public domain.

import cv2
from configuration import *


def resize_calculate(a, b, p):
    after_percent = p / 100
    remain_percent = ((100 - p) / 2) / 100
    c = b * remain_percent
    d = b * (after_percent + remain_percent)
    return int(c), int(d)


def snippet_rgb(inner_width, percent_a, inner_height, percent_b):
    data0, data1 = resize_calculate(0, inner_width, percent_a)
    data2, data3 = resize_calculate(0, inner_height, percent_b)
    return data0, data1, data2, data3


def center_data_compression(frame):
    compressed = cv2.resize(frame, capabilities['camera']["central_vision_compression"], interpolation=cv2.INTER_AREA)
    return compressed


def peripheral_data_compression(frame):
    compressed = cv2.resize(frame, capabilities['camera']["peripheral_vision_compression"],
                            interpolation=cv2.INTER_AREA)
    return compressed


def frame_split(frame):
    full_data = frame.shape
    width_data1, width_data2, height_data1, height_data2 = snippet_rgb(full_data[0],
                                                                       capabilities['camera']['retina_width_percent'],
                                                                       full_data[1],
                                                                       capabilities['camera']['retina_height_percent'])
    print("C and D for width: ", width_data1, " ", width_data2)
    print("C and D for height: ", height_data1, " ", height_data2)
    TL = frame[0:width_data1, 0:height_data1]
    TM = frame[0:width_data1, height_data1:height_data2]
    TR = frame[0:width_data1, height_data2:]
    ML = frame[width_data1:width_data2, 0:height_data1]
    C = frame[width_data1:width_data2, height_data1:height_data2]
    C = center_data_compression(C)
    MR = frame[width_data1:width_data2, height_data2:]
    LL = frame[width_data2:, 0:height_data1]
    LM = frame[width_data2:, height_data1: height_data2]
    LR = frame[width_data2:, height_data2:]
    TL = peripheral_data_compression(TL)
    TM = peripheral_data_compression(TM)
    TR = peripheral_data_compression(TR)
    ML = peripheral_data_compression(ML)
    MR = peripheral_data_compression(MR)
    LL = peripheral_data_compression(LL)
    LM = peripheral_data_compression(LM)
    LR = peripheral_data_compression(LR)
    cv2.imshow("TL", TL)
    cv2.imshow("TM", TM)
    cv2.imshow("TR", TR)
    cv2.imshow("ML", ML)
    cv2.imshow("C", C)
    cv2.imshow("MR", MR)
    cv2.imshow("LL", LL)
    cv2.imshow("LM", LM)
    cv2.imshow("LR", LR)


img = cv2.imread('goku.jpeg', cv2.IMREAD_UNCHANGED)

print('Original Dimensions : ', img.shape)

# scale_percent = 10  # percent of original size
width = 720
height = 960
dim = (width, height)
# resize image
resized = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)
print('Resized Dimensions : ', resized.shape)

# PRACTICE TO GET DATA SNIPPET
bracket = dict()
Full_data = resized
snippet = Full_data[0:280, 0:320]
# cv2.imshow("test", snippet)
bracket['TL'] = snippet

Full_data = resized
snippet = Full_data[0:280, 320:576]
# cv2.imshow("test", snippet)
bracket['TM'] = snippet

Full_data = resized
snippet = Full_data[0:280, 576:960]
# cv2.imshow("test", snippet)
bracket['TR'] = snippet

Full_data = resized
snippet = Full_data[280:420, 0:320]
# cv2.imshow("test", snippet)
bracket['ML'] = snippet

Full_data = resized
snippet = Full_data[280:420, 320:576]
# cv2.imshow("test", snippet)
bracket['C'] = snippet

Full_data = resized
snippet = Full_data[280:420, 576:960]
# cv2.imshow("test", snippet)
bracket['MR'] = snippet

Full_data = resized
snippet = Full_data[420:700, 0:320]
# cv2.imshow("test", snippet)
bracket['LL'] = snippet

Full_data = resized
snippet = Full_data[420:700, 320:576]
# cv2.imshow("test", snippet)
bracket['LM'] = snippet

Full_data = resized
snippet = Full_data[420:700, 576:960]
# cv2.imshow("test", snippet)
bracket['LR'] = snippet

# data0, data1 = resize_calculate(0, 100, 10)
# print("C: ", data0, " D: ", data1)
snippet_rgb(700, 20, 960, 15)
frame_split(resized)

# cv2.imshow("TL", bracket['TL'])
# cv2.imshow("TM", bracket['TM'])
# cv2.imshow("TR", bracket['TR'])
# cv2.imshow("ML", bracket['ML'])
# cv2.imshow("C", bracket['C'])
# cv2.imshow("MR", bracket['MR'])
# cv2.imshow("LL", bracket['LL'])
# cv2.imshow("LM", bracket['LM'])
# cv2.imshow("LR", bracket['LR'])
cv2.imshow("sss", resized)

# cv2.imshow("Resized image", resized)
cv2.waitKey(0)
cv2.destroyAllWindows()
