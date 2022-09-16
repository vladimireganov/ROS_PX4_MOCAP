

# file for visualization of incoming data stream

import cv2

import numpy as np

import pandas as pd

 

Kd = 0.003

Ki = 0.111

Kp = 0.19

throttle = 0.12

 

def visualizer(img):

    cv2.imshow("image", img)

 

def nothing(x):

    pass

 

def create_track_bars():

    cv2.namedWindow('controls',2)

    cv2.createTrackbar('u','controls',0,200,nothing)

    cv2.createTrackbar('servo','controls',0,200,nothing)

    cv2.createTrackbar('Kd','controls',0,1000,nothing)

    cv2.createTrackbar('Ki','controls',0,10000,nothing)

    cv2.createTrackbar('Kp','controls',0,10000,nothing)

    cv2.createTrackbar('throttle','controls',0,3500,nothing)

    cv2.setTrackbarMin('u','controls',-200)

 

    cv2.setTrackbarPos('Kp','controls',int(Kp * 1000))

    cv2.setTrackbarPos('Kd','controls',int(Kd * 10000))

    cv2.setTrackbarPos('Ki','controls',int(Ki * 1000))

    cv2.setTrackbarPos('throttle','controls',int(throttle * 1000))
