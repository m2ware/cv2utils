#!/usr/bin/python3

from picamera import PiCamera
from cv2utils.tracker2 import Tracker, Subscriber, Handler, MotionProcessor, EventDetector
from cv2utils.tracker2 import save_image_handler
from cv2utils.motorcontroller import MotorController
import cv2utils.cv2utils as cvu
import os
import numpy as np
import time
import logging

# Steer to neutral
os.system("gpio_pwm 20 10000 25 " + str(15.0) + " &" )
os.system("gpio_pwm 21 10000 25 " + str(15.0) + " &" )
time.sleep(1.5)

# Create a new tracker object for Video0 (USB camera)
tracker = Tracker(usb_dev=0,
                  vflip=True, hflip=True)

# Create a subscriber for detecting motion and steering the camera
# My laser is a little bit off-center, hence the bias adjustment...
motor_controller = MotorController(bias_x=-37, bias_y=10)

detector = EventDetector(time_between_triggers_s=0.85,
                         min_sequential_frames=2,
                         min_contour_area_px=200,
                         max_contour_area_px=50000)

subscriber = Subscriber(handler=motor_controller, event_detector=detector,
                        name="Motor Tracker")
tracker.add_subscriber(subscriber)

# Create a subscriber for detecting motion and saving an image
detector = EventDetector(time_between_triggers_s=10.0,
                         min_sequential_frames=1,
                         min_contour_area_px=200,
                         max_contour_area_px=50000)
subscriber = Subscriber(handler=save_image_handler, event_detector=detector,
                        name="Save Image")
# Comment / uncomment to save periodic captures.
tracker.add_subscriber(subscriber)

# Uncomment to turn off debug-level logging
#log = Tracker.get_logger()
#log.level = logging.INFO

# Run with USB cam
tracker.run_usb()
