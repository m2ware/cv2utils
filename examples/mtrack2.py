#!/usr/bin/python3

from picamera import PiCamera
from cv2utils.tracker import Tracker, DetectionEvent, save_motion_image_handler
import cv2utils.cv2utils as cvu
import os
import numpy as np
import time

rez = (320, 240)

#camera = PiCamera()
#camera.hflip = True
#camera.vflip = True
#camera.resolution = rez
#camera.framerate = 8

def motor_handler(contours, images, state):
    
    imsize = images[0].shape
    print(imsize)
    hres=imsize[1]/2.
    vres=imsize[0]/2.
    # Find area of largest motion contour
    contour, area = cvu.largest_contour(contours)
    centroid = cvu.centroid(contour)
    # Normalize the largest motion centroid in the [-1,1] space
    hpos = (centroid[0]-hres)/hres
    vpos = (centroid[1]-vres)/vres
    # Convert to number of PWM pulses 
    max_ticks = 3
    hdelta = max_ticks*hpos
    vdelta = max_ticks*vpos
    
    mindelta = 0.05
    if (np.abs(hdelta) < mindelta) and (np.abs(vdelta) < mindelta) : return
    
    state.xpos -= hdelta; 
    state.ypos += vdelta
    if (state.xpos > 22): state.xpos = 22
    if (state.xpos < 8): state.xpos = 8
    if (state.ypos > 22): state.ypos = 22
    if (state.ypos < 8): state.ypos = 8
    
    # These are hard-coded motor positions for L/R and U/D
    # 15 corresponds to 1500us position (neutral) on standard servo
    # Stuff for debug
    print("N=" + str(len(contours)))
    print("A=" + str(area))
    print("centroid = " + str(centroid))
    print("hdelta=" + str(np.round(hdelta,2)) + 
          ", vdelta=" + str(np.round(vdelta,2)))
    print("htgt= " + str(np.round(state.xpos,2)) + 
          ", vtgt= " + str(np.round(state.ypos,2)) )

    # Call command-line for PWM pulse train generator as detached processes
    # to steer incrementally toward detected motion.  
    os.system("gpio_pwm 20 10000 2 " + 
              str(np.round(state.xpos,2)) + " &" )
    os.system("gpio_pwm 21 10000 2 " + 
              str(np.round(state.ypos,2)) + " &" )

# Steer to neutral
state = type('state', (object,), {})()
state.xpos = 15.0
state.ypos = 15.0
os.system("gpio_pwm 20 10000 50 " + str(state.xpos) + " &")
os.system("gpio_pwm 21 10000 50 " + str(state.ypos) + " &")
time.sleep(1.0)

# Create a new tracker object for Video0 (USB camera)
tracker = Tracker(camera=None, res=rez, usb_dev=0, threshold=45, 
                  vflip=True, hflip=True)
# Clear out the default chatty detection handler
tracker.events = []

# Add an event handler to save images on sequential movement frames
event = DetectionEvent(handler=save_motion_image_handler,
                       time_between_triggers_s = 5.0,
                       min_sequential_frames=3,
                       min_contour_area_px=600)
# Comment/uncomment to activate
#tracker.add_event(event)

# Add the servo-steering event / handler
event = DetectionEvent(handler=motor_handler,
                       time_between_triggers_s = 0.75,
                       min_sequential_frames=2,
                       min_contour_area_px=600, state=state)
# Comment/uncomment to activate
tracker.add_event(event)

print("Starting tracker...")

# This one is for RaspiCam.  
#tracker.run()
# This one is for USB cam
tracker.run_usb()
