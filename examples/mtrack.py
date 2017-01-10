#!/usr/bin/python3

from picamera import PiCamera
from cv2utils.tracker import Tracker, DetectionEvent, save_motion_image_handler
import cv2utils.cv2utils as cvu
import os
import numpy as np

rez = (320, 240)

#camera = PiCamera()
#camera.hflip = True
#camera.vflip = True
#camera.resolution = rez
#camera.framerate = 8

def motor_handler(contours, images):
    hres=320
    vres=240
    contour, area = cvu.largest_contour(contours)
    print("A=" + str(area))
    centroid = cvu.centroid(contour)
    hpos = (centroid[0]-hres)/hres
    vpos = (centroid[1]-vres)/vres
    max_ticks = 5.0
    hticks = np.round(max_ticks*np.abs(hpos))
    vticks = np.round(max_ticks*np.abs(vpos))
    if (hpos < 0): htgt = 22 
    else: htgt = 9
    if (vpos > 0): vtgt = 22
    else: vtgt = 9
    print(centroid)
    print(hpos)
    print(vpos)
    print("h=" + str(htgt) + ", v=" + str(vtgt));
    print("hticks=" + str(hticks) + ", vticks=" + str(vticks))

    os.system("gpio_pwm 20 10000 " + str(hticks) + " " + str(htgt) + " &" )
    os.system("gpio_pwm 21 10000 " + str(vticks) + " " + str(vtgt) + " &" )

tracker = Tracker(camera=None, res=rez, usb_dev=0, threshold=55, 
                  vflip=True, hflip=True)
tracker.events = []

# Add an event handler to save images on 3 sequential movement frames
event = DetectionEvent(handler=save_motion_image_handler,
                       time_between_triggers_s = 5.0,
                       min_sequential_frames=2,
                       min_contour_area_px=500)

#tracker.add_event(event)

event = DetectionEvent(handler=motor_handler,
                       time_between_triggers_s = 0.75,
                       min_sequential_frames=3,
                       min_contour_area_px=500)
tracker.add_event(event)

print("Starting tracker...")

#tracker.run()
tracker.run_usb()
