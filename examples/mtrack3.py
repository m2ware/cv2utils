#!/usr/bin/python3

from picamera import PiCamera
from cv2utils.tracker2 import Tracker, Subscriber, Handler, MotionProcessor, EventDetector
from cv2utils.tracker2 import save_image_handler
from cv2utils.motorcontroller import MotorController
from cv2utils.espeakcontroller import EspeakController
from pibotspeech import speech_items
import cv2utils.cv2utils as cvu
import os
import numpy as np
import time
import logging

# Steer to neutral
time.sleep(0.5)
os.system("gpio_pwm 20 10000 25 15 & " )
os.system("gpio_pwm 21 10000 25 15 & " )
time.sleep(1.0)

# Create a new tracker object for Video0 (USB camera)
tracker = Tracker(usb_dev=0,
                  vflip=True, hflip=True)

# Create a subscriber for detecting motion and steering the camera
# My laser is a little bit off-center, hence the bias adjustment...
motor_controller = MotorController(bias_x=-37, bias_y=10, steering_gain=4.0)

detector = EventDetector(time_between_triggers_s=0.9,
                         min_sequential_frames=2,
                         min_contour_area_px=500,
                         max_contour_area_px=50000)

subscriber = Subscriber(handler=motor_controller, event_detector=detector,
                        name="Motor Tracker")
tracker.add_subscriber(subscriber)

# Create a subscriber for detecting motion and saving an image
# This one has a built-in 60s delay between saves
# Note that if you leave this on it can fill up your disk in a hurry with
# Saved image frames!!!
detector = EventDetector(time_between_triggers_s=30.0,
                         min_sequential_frames=1,
                         min_contour_area_px=500,
                         max_contour_area_px=50000)
subscriber = Subscriber(handler=save_image_handler, event_detector=detector,
                        name="Save Image")
# Comment / uncomment to save periodic captures.
#tracker.add_subscriber(subscriber)
espeak_controller = EspeakController(speech_items=speech_items)
detector = EventDetector(time_between_triggers_s=6,
                         min_sequential_frames=2,
                         min_contour_area_px=8000,
                         max_contour_area_px=100000)
subscriber = Subscriber(handler=espeak_controller, event_detector=detector,
                        name="Espeak")
# Comment / uncomment to make robot talk
tracker.add_subscriber(subscriber)

# Uncomment to turn off debug-level logging
log = Tracker.get_logger()
log.level = logging.INFO
#log.level = logging.DEBUG
# Run with USB cam
tracker.run_usb()
