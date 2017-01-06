#!/usr/bin/python3

# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import sys
import cv2utils.cv2utils as cvu

writeComposite = True

for arg in sys.argv:
    print(str(arg))
    if arg == "nowrite": 
        print("Deactivating composite write")
        writeComposite = False
 
# initialize the camera and grab a reference to the raw camera capture
print("Initializing piCam")
rez = (640,480)
camera = PiCamera()
camera.hflip = True
camera.vflip = True
camera.resolution = rez
raw1 = PiRGBArray(camera)
raw2 = PiRGBArray(camera) 

# allow the camera to warmup
print("warmup...")
time.sleep(0.1)

# Get a dump frame just in case
raw0 = PiRGBArray(camera)
camera.capture(raw0, format="rgb")
time.sleep(0.25)
 
# grab an image from the camera
print("grabbing image 1...")
camera.capture(raw1, format="rgb")
im1 = raw1.array

#time.sleep(0.05)

print("grabbing image 2...")
camera.capture(raw2, format="rgb")
im2 = raw2.array

#diff = np.abs(imdiff(im1, im2))
#diff_gray = cv2.cvtColor(diff, cv2.COLOR_RGB2GRAY)
diff, diff_gray = cvu.frame_diff(im1, im2)
contours, mask = cvu.get_contours(diff_gray, thresh=25, erode=False)
print("found " + str(len(contours)) + " movement contours.")
cvu.contour_area(contours)

cvu.hist_print(diff_gray, bins=10)
cvu.hist_print(im2, bins=10)

print("im1.shape = " + str(im1.shape));
print("im2.shape = " + str(im2.shape));
print("diff.shape = " + str(diff.shape));

result = cvu.get_motion_image( (im1, im2, diff), contours )

print("res.shape = " + str(result.shape));

# Save to a file
if writeComposite:
    filename = cvu.imwrite_timestamp(result, prefix="diff_")
    print("writing " + filename)

print("Done!")

# display the image on screen and wait for a keypress
#cv2.imshow("Image", image)
#cv2.waitKey(0)
