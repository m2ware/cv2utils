#!/usr/bin/python3

from picamera import PiCamera
from cv2utils.tracker2 import Tracker, Subscriber, Handler, MotionProcessor, EventDetector
import cv2utils.cv2utils as cvu
import os
import numpy as np
import time
import logging

class MotorController(Handler):

    def __init__(self, bias_x=0, bias_y=0, h_pin=20, v_pin=21):

        self.xpos = 15.0
        self.ypos = 15.0
        self.bias_x = bias_x
        self.bias_y = bias_y
        self.h_pin = h_pin
        self.v_pin = v_pin

    def handle(self, contours, frame_buf, frame_index):

        log = Tracker.get_logger()
        imsize = frame_buf[frame_index].shape
        hres=imsize[1]/2.
        vres=imsize[0]/2.

        #biasx = -37
        #biasy = 10

        tgtx = hres+self.bias_x
        tgty = vres+self.bias_y
        # Find area of largest motion contour
        #contour, area = cvu.largest_contour(contours)
        #centroid = cvu.centroid(contour)
        centroid = cvu.avg_contour_centroid(contours)

        # Normalize the largest motion centroid in the [-1,1] space
        hpos = (centroid[0]-tgtx)/hres
        vpos = (centroid[1]-tgty)/vres
        # Convert to number of PWM pulses
        max_delta = 13.0
        hdelta = max_delta*(hpos*np.abs(hpos))
        vdelta = max_delta*(vpos*np.abs(vpos))
        h_pulses = int(np.ceil(np.abs(hdelta)))+1
        v_pulses = int(np.ceil(np.abs(vdelta)))+1
        log.debug("hpc=" + str(h_pulses) + ", vpc=" + str(v_pulses))

        mindelta = 0.05

        self.xpos -= hdelta;
        self.ypos += vdelta
        if (self.xpos > 22): self.xpos = 22
        if (self.xpos < 8): self.xpos = 8
        if (self.ypos > 22): self.ypos = 22
        if (self.ypos < 8): self.ypos = 8

        # These are hard-coded motor positions for L/R and U/D
        # 15 corresponds to 1500us position (neutral) on standard servo
        # Stuff for debug
        log.debug("N=" + str(len(contours)))
        #log.debug("A=" + str(area))
        log.debug("centroid = " + str(centroid))
        log.debug("hdelta=" + str(np.round(hdelta,2)) +
                ", vdelta=" + str(np.round(vdelta,2)))
        log.debug("htgt= " + str(np.round(self.xpos,2)) +
                ", vtgt= " + str(np.round(self.ypos,2)) )

        # Call command-line for PWM pulse train generator as detached processes
        # to steer incrementally toward detected motion.
        self.servo_steer(self.h_pin, h_pulses, self.xpos)
        self.servo_steer(self.v_pin, v_pulses, self.ypos)

    def servo_steer(self, pin, pulses, position):
        os.system("gpio_pwm " + str(pin) + " 10000 " + str(pulses) + " " +
                  str(position) + " &")


# Steer to neutral
os.system("gpio_pwm 20 10000 25 " + str(15.0) + " &" )
os.system("gpio_pwm 21 10000 25 " + str(15.0) + " &" )
time.sleep(1.5)

# Create a new tracker object for Video0 (USB camera)
tracker = Tracker(usb_dev=0,
                  vflip=True, hflip=True)
# Clear out the default chatty detection handler
#tracker.events = []
motor_controller = MotorController(bias_x=-37, bias_y=10)

detector = EventDetector(time_between_triggers_s=0.85,
                         min_sequential_frames=2,
                         min_contour_area_px=200,
                         max_contour_area_px=50000)


subscriber = Subscriber(handler=motor_controller, event_detector=detector,
                        name="Motor Tracker")
tracker.add_subscriber(subscriber)

# Uncomment to turn off debug-level logging
#log = Tracker.get_logger()
#log.level = logging.INFO

# Add an event handler to save images on sequential movement frames
#event = DetectionEvent(handler=save_motion_image_handler,
#                       time_between_triggers_s = 10.0,
#                       min_sequential_frames=1,
#                       min_contour_area_px=200)
# Comment/uncomment to activate
#tracker.add_event(event)

# Add the servo-steering event / handler
#event = DetectionEvent(handler=motor_handler,
#                       time_between_triggers_s = 0.8,
#                       min_sequential_frames=2,
#                       min_contour_area_px=200, state=state)
# Comment/uncomment to activate
#tracker.add_event(event)

# This one is for RaspiCam.
#tracker.run()
# This one is for USB cam
tracker.run_usb()
