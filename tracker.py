#!/usr/bin/python3

#imports
import cv2
import time
import numpy as np
#from picamera.array import PiRGBArray
#from picamera import PiCamera
import logging

import cv2utils.cv2utils as cvu

class Tracker:

    def __init__(self, camera, res=(640,480), farr_len=2,
                 heartbeat_frames=250, usb_dev=0, threshold=25,
                 vflip=False, hflip=False):
        self._camera = camera
        self.farr_len = farr_len
        self.frame_array = [None] * self.farr_len
        self.res = res
        self._fr_count = 0
        self._hb_time = 0
        self._heartbeat_frames = heartbeat_frames
        self.usb_dev = usb_dev
        self.vflip = vflip
        self.hflip = hflip
        self.threshold=threshold
        self.events = list()
        self.events.append(DetectionEvent())
        self._logger = Tracker._get_default_logger()
        print(__name__)

    def _farr_idx(self, prev_frame=False):
        if (prev_frame):
            return (self._fr_count-1)%self.farr_len
        return self._fr_count % self.farr_len

    def _store_frame(self, img):
        if (self.vflip):
            img=cv2.flip(img,0)
        if (self.hflip):
            img=cv2.flip(img,1)
        self.frame_array[self._farr_idx()] = img
        self._fr_count += 1

    def _detect_motion(self):
        img1 = self.frame_array[self._farr_idx()]
        img2 = self.frame_array[self._farr_idx(prev_frame=True)]
        diff, diff_gray = cvu.frame_diff(img1, img2)
        contours, mask = cvu.get_contours(diff_gray, thresh=self.threshold, 
                                          erode=True, dilate=True)
        images = (img1, img2, diff)
        self._process_events(contours, images)

    def _process_events(self, contours, images):
        for event in self.events:
            event.detect(contours, images)

    def get_logger():
        log = logging.getLogger(__name__)
        return log

    def _heartbeat(self):
        log = Tracker.get_logger()
        if ( self._fr_count % self._heartbeat_frames ) == 0:
            now = time.time()
            fps = self._heartbeat_frames / (now-self._hb_time);
            self._hb_time = now
            log.info("[heartbeat] fr=" + str(self._fr_count) +
                  " fps=" + str(np.round(fps,2)))

    def _process_frame(self, frame):
        self._store_frame(frame)
        self._detect_motion()
        self._heartbeat()

    def run_usb(self):
        log = Tracker.get_logger()
        log.info("Started tracking on video" + str(self.usb_dev))
        # Initialize with one frame
        cap = cv2.VideoCapture(self.usb_dev)
        self._hb_time = time.time()
        ret, frame = cap.read()
        log.info("Resolution = " + str(frame.shape))

        self._store_frame(frame)

        while(True):
            ret, frame = cap.read()
            self._process_frame(frame)

#    def run(self):
#        log = Tracker.get_logger()
#        log.info("Started tracking on RaspiCam")
        # Initialize with one frame
#        self._hb_time = time.time()
#        rawCapture = PiRGBArray(self._camera, size=self.res)
#        self._camera.capture(rawCapture, format="bgr")
#        log.info("Resolution = " + str(rawCapture.array.shape))

#        self._store_frame(rawCapture.array)
#        rawCapture.truncate(0)

#        for frame in self._camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
#            self._process_frame(frame)
#            rawCapture.truncate(0)

    def add_event(self, event):
        log = Tracker.get_logger()
        # Should add ID to events for logging clarity...
        log.info("Added detection event")
        self.events.append(event)

    def _get_default_logger():
        logging.basicConfig(level=logging.DEBUG,
                            format='[%(asctime)s][%(levelname)s]%(message)s',
                            datefmt='%y-%m-%d %I:%M:%S',
                            filename='Tracker.log',
                            filemode='a')
        console = logging.StreamHandler()
        console.setLevel(logging.DEBUG)
        formatter = logging.Formatter('[%(asctime)s][%(levelname)s]%(message)s')
        console.setFormatter(formatter)
        logging.getLogger(__name__).addHandler(console)

#  Useful handler functions

#  Print detection message to screen
def default_handler(contours, images, state=None):
    log = logging.getLogger(__name__)
    log.debug("Motion detected in " + str(len(contours)) + " regions!")
    contour, area = cvu.largest_contour(contours)
    log.debug("Largest centroid: " + str(cvu.centroid(contour)) + ", A=" + str(area))

#  Print detection message and save image showing detected motion contours
def save_motion_image_handler(contours, images, state=None):
    log = logging.getLogger(__name__)
    default_handler(contours, images)
    contour, area = cvu.largest_contour(contours)
    #centroid = cvu.centroid(contour)
    centroid = cvu.avg_contour_centroid(contours)

    log.debug("c = " + str(centroid))
    # Place the detected contours on the images
    for image in images:
        cv2.drawContours(image, contours, -1,
                         color=(25,128,255), thickness=1)
        cvu.draw_x(images[0],centroid,length=9)

    result = cvu.get_motion_image(images, contours)
    filename = cvu.imwrite_timestamp(result, prefix="Event_")
    log.info("Wrote " + filename)

class DetectionEvent:

    def __init__(self, handler=default_handler,
                 time_between_triggers_s=1.0,
                 min_sequential_frames=1,
                 min_contour_area_px=50, state=None):
        self.handler = handler
        self._last_event_time = 0
        self._trigger_count = 0
        self.time_between_triggers_s = time_between_triggers_s
        self.min_contour_area_px = min_contour_area_px
        self.min_sequential_frames = min_sequential_frames
        self._state = state

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
            self.handler(contours, images, self._state)
            self._trigger_count = 0
            self._last_event_time = now
            return True

    def _meets_min_area_contour(self, contour):
        if cv2.contourArea(contour) >= self.min_contour_area_px:
            return true

    def _meets_min_area(self, contours):
        for contour in contours:
            if cv2.contourArea(contour) >= self.min_contour_area_px :
                return True
        return False

