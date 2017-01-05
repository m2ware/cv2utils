#!/usr/bin/python3

from picamera import PiCamera
from cv2utils.tracker import Tracker

rez = (320, 240)

camera = PiCamera()
camera.hflip = True
camera.vflip = True
camera.resolution = rez
camera.framerate = 8

tracker = Tracker(camera=camera, res=rez)

print("Starting tracker...")

tracker.run()
