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
    # These are hard-coded right now for 640x480
    hres=320
    vres=240
    # Find area of largest motion contour
    contour, area = cvu.largest_contour(contours)
    centroid = cvu.centroid(contour)
    # Normalize the largest motion centroid in the [-1,1] space
    hpos = (centroid[0]-hres)/hres
    vpos = (centroid[1]-vres)/vres
    # Convert to number of PWM pulses 
    max_ticks = 4.0
    hticks = np.round(max_ticks*np.abs(hpos))
    vticks = np.round(max_ticks*np.abs(vpos))

    #hticks = 0; vticks = 0    
    #if (np.abs(hpos))>0.1: hticks = 1
    #if (np.abs(vpos))>0.1: vticks = 1
    
    # These are hard-coded motor positions for L/R and U/D
    # 15 corresponds to 1500us position (neutral) on standard servo
    if (hpos < 0): htgt = 22 
    else: htgt = 9
    if (vpos > 0): vtgt = 22
    else: vtgt = 9
    # Stuff for debug
    print("N=" + str(len(contours)))
    print("A=" + str(area))
    print(centroid)
    print("hpos= " + str(hpos) + ", vpos= " + str(vpos))
    print("hticks=" + str(hticks) + ", vticks=" + str(vticks))
    print("htgt= " + str(htgt) + ", vtgt= " + str(vtgt))

    # Call command-line for PWM pulse train generator as detached processes
    # to steer incrementally toward detected motion.  
    os.system("gpio_pwm 20 10000 " + str(hticks) + " " + str(htgt) + " &" )
    os.system("gpio_pwm 21 10000 " + str(vticks) + " " + str(vtgt) + " &" )

# Create a new tracker object for Video0 (USB camera)
tracker = Tracker(camera=None, res=rez, usb_dev=0, threshold=55, 
                  vflip=True, hflip=True)
# Clear out the default chatty detection handler
tracker.events = []

# Add an event handler to save images on sequential movement frames
event = DetectionEvent(handler=save_motion_image_handler,
                       time_between_triggers_s = 5.0,
                       min_sequential_frames=5,
                       min_contour_area_px=500)
# Comment/uncomment to activate
#tracker.add_event(event)

# Add the servo-steering event / handler
event = DetectionEvent(handler=motor_handler,
                       time_between_triggers_s = 1.1,
                       min_sequential_frames=3,
                       min_contour_area_px=500)
# Comment/uncomment to activate
tracker.add_event(event)

print("Starting tracker...")

# This one is for RaspiCam.  
#tracker.run()
# This one is for USB cam
tracker.run_usb()
