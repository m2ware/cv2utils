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

rawCapture = PiRGBArray(camera)

# allow the camera to warmup
print("warmup...")
time.sleep(0.1)

# grab an image from the camera
print("grabbing image...")
camera.capture(rawCapture, format="rgb")
image = rawCapture.array

# Save to a file
filename = time.strftime("%Y-%m-%d_%H_%M_%S") + ".jpeg"
print("writing " + filename)
cv2.imwrite(filename, image)

print("Done!")
# display the image on screen and wait for a keypress
#cv2.imshow("Image", image)
#cv2.waitKey(0)
