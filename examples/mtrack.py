#!/usr/bin/python3

from picamera import PiCamera
from cv2utils.tracker import Tracker, DetectionEvent, save_motion_image_handler

rez = (320, 240)

camera = PiCamera()
camera.hflip = True
camera.vflip = True
camera.resolution = rez
camera.framerate = 8

tracker = Tracker(camera=camera, res=rez)
# Add an event handler to save images on 3 sequential movement frames
event = DetectionEvent(handler=save_motion_image_handler,
                       time_between_triggers_s = 3.0,
                       min_sequential_frames=2,
                       min_contour_area_px=200)

tracker.add_event(event)

print("Starting tracker...")

tracker.run()
