#!/usr/bin/python3

#imports
import cv2
import time
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import logging

import cv2utils.cv2utils as cvu

class Tracker:

    def __init__(self, camera, res=(640,480), farr_len=2, heartbeat_frames=250):
        self._camera = camera
        self.farr_len = farr_len
        self.frame_array = [None] * self.farr_len
        self.res = res
        self._fr_count = 0
        self._hb_time = 0
        self._heartbeat_frames = heartbeat_frames
        self.events = list()
        self.events.append(DetectionEvent())

    def _farr_idx(self, prev_frame=False):
        if (prev_frame): 
            return (self._fr_count-1)%self.farr_len
        return self._fr_count % self.farr_len
    
    def _store_frame(self, img):
        self.frame_array[self._farr_idx()] = img
        self._fr_count += 1
        
    def _detect_motion(self):
        print(self._farr_idx());
        img1 = self.frame_array[self._farr_idx()]
        img2 = self.frame_array[self._farr_idx(prev_frame=True)]
        diff, diff_gray = cvu.frame_diff(img1, img2)
        contours, mask = cvu.get_contours(diff_gray, thresh=25, erode=False)
        images = (img1, img2, diff)
        self._process_events(contours, images)
        
    def _process_events(self, contours, images):
        for event in self.events:
            event.detect(contours, images)
        
    def _heartbeat(self):
        if ( self._fr_count % self._heartbeat_frames ) == 0:
            now = time.time()
            fps = self._heartbeat_frames / (now-self._hb_time);
            self._hb_time = now
            timestamp = time.strftime("%Y-%m-%d %I:%M:%S")
            print("[" + timestamp + "] fr= " + str(self._fr_count) + 
                  " fps=" + str(np.round(fps,2)))
        
    def run(self):
        # Initialize with one frame
        self._hb_time = time.time()
        rawCapture = PiRGBArray(self._camera, size=self.res)
        self._camera.capture(rawCapture, format="bgr")
        self._store_frame(rawCapture.array)
        rawCapture.truncate(0)

        for frame in self._camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            self._store_frame(frame.array)
            self._detect_motion()
            self._heartbeat()
            rawCapture.truncate(0)
            
    def add_event(self, event):
        self.events.append(event)

#  Useful handler functions            

#  Print detection message to screen
def default_handler(contours, images):
    print("Motion detected in " + str(len(contours)) + " regions!")
    contour, area = cvu.largest_contour(contours)
    print("Largest centroid at " + str(cvu.centroid(contour)) + ", A=" + str(area))

#  Print detection message and save image showing detected motion contours
def save_motion_image_handler(contours, images):
    default_handler(contours, images)
    result = cvu.get_motion_image(images, contours)
    filename = cvu.imwrite_timestamp(result, prefix="Event_")
    print("wrote " + filename)

class DetectionEvent:

    def __init__(self, handler=default_handler,
                 time_between_triggers_s=1.0,
                 min_sequential_frames=1,
                 min_contour_area_px=50):
        self.handler = handler
        self._last_event_time = 0
        self._trigger_count = 0
        self.time_between_triggers_s = time_between_triggers_s
        self.min_contour_area_px = min_contour_area_px
        self.min_sequential_frames = min_sequential_frames

    # Apply detection criteria and invoke handler if satisfied
    def detect(self, contours, images):
        if len(contours) > 0 and self._meets_min_area(contours):
            self._trigger_count +=1
        else:
            self._trigger_count = 0
            return False
        now = time.time()
        if (now - self._last_event_time) < self.time_between_triggers_s :
            return False
        if self._trigger_count >= self.min_sequential_frames :
            self.handler(contours, images)
            self._trigger_count = 0
            self._last_event_time = now
            return True

    def _meets_min_area(self, contours):
        for contour in contours:
            if cv2.contourArea(contour) >= self.min_contour_area_px :
                return True
        return False

