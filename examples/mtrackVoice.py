#!/usr/bin/python3

from picamera import PiCamera
from cv2utils.tracker import Tracker, Handler, EventDetector
from cv2utils.frameprocessor import MotionProcessor, ColorDetector
from cv2utils.tracker import save_image_handler
from cv2utils.subscriber import Subscriber
from cv2utils.motorcontroller import MotorController
from cv2utils.espeakcontroller import EspeakController
from speech import speech_items
#import cv2utils.cv2utils as cvu
import os
import time
import logging

# Steer to neutral
time.sleep(0.5)
os.system("gpio_pwm 20 10000 25 15 & " )
os.system("gpio_pwm 21 10000 25 14 & " )
time.sleep(1.0)

#colorbounds = ([135, 83, 23], [253, 198, 56])
#colorbounds = ([9, 80, 50], [30, 225, 220])
colorbounds = ([107, 70, 45], [114, 255, 225])
# frame_proc = ColorDetector(bounds = colorbounds )
frame_proc = MotionProcessor()

# Create a new tracker object for Video0 (USB camera)
tracker = Tracker(usb_dev=0,
                  vflip=True, hflip=True)

# Create a subscriber for detecting motion and steering the camera
# My laser is a little bit off-center, hence the bias adjustment...
motor_controller = MotorController(bias_x=-10, bias_y=15,
                                   steering_gain=3.5,
                                   target_global_centroid=False)

detector = EventDetector(time_between_triggers_s=0.85,
                         min_sequential_frames=2,
                         min_contour_area_px=500,
                         max_contour_area_px=50000)

subscriber = Subscriber(handler=motor_controller,
                        frame_processor=frame_proc,
                        event_detector=detector,
                        name="Motor Tracker")
tracker.add_subscriber(subscriber)

# Create a subscriber for detecting motion and saving an image
# This one has a built-in 60s delay between saves
# Note that if you leave this on it can fill up your disk in a hurry with
# Saved image frames!!!
detector = EventDetector(time_between_triggers_s=30.0,
                         min_sequential_frames=2,
                         min_contour_area_px=400,
                         max_contour_area_px=50000)
subscriber = Subscriber(handler=save_image_handler,
                        frame_processor=frame_proc,
                        event_detector=detector,
                        name="Save Image")
# Comment / uncomment to save periodic captures.
# tracker.add_subscriber(subscriber)

flags = '-v en+f3 -p 50 -s 175'
espeak_controller = EspeakController(speech_items=speech_items, flags=flags)
detector = EventDetector(time_between_triggers_s=6,
                         min_sequential_frames=2,
                         min_contour_area_px=400,
                         max_contour_area_px=100000)
subscriber = Subscriber(handler=espeak_controller,
                        frame_processor=frame_proc,
                        event_detector=detector,
                        name="Espeak")
# Comment / uncomment to make robot talk
#tracker.add_subscriber(subscriber)

# Uncomment to turn off debug-level logging
log = Tracker.get_logger()
# log.level = logging.INFO
log.level = logging.DEBUG
# Run with USB cam
tracker.run_usb()

while (True):

    log.info('Main thread running.')
    #print('main running thread')
    time.sleep(60)
