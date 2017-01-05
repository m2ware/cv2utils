#!/usr/bin/python3

# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
 
# initialize the camera and grab a reference to the raw camera capture
print("Initializing piCam")
camera = PiCamera()
camera.hflip = True
camera.vflip = True
#rez = (320, 240)
rez = (640,480)
camera.resolution = rez
camera.framerate = 32
 
# allow the camera to warmup
print("warmup...")
time.sleep(0.1)
 
# grab an image from the camera
print("polling camera... CTRL<C> to quit")
N = 10
i = 0
start = time.time()
rawCapture = PiRGBArray(camera, size=rez)

while True:

    #for i in range(0,N):
    camera.capture(rawCapture, format="bgr")
    image = rawCapture.array
    rawCapture.truncate(0)
    
    i += 1

    if (i == N):
        end = time.time()
        delta_t = end-start
        print("Captured " + str(N) + " frames in " + str(delta_t))
        print("  (" + str(N/delta_t) + " fps)" )
        i = 0
        start = time.time()

print("Done!")
