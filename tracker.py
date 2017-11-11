#!/usr/bin/python3

#imports
import cv2
import time
import numpy as np
#from picamera.array import PiRGBArray
#from picamera import PiCamera
import logging
import types
from threading import Thread

import cv2utils.cv2utils as cvu
#from cv2utils.subscriber import Subscriber
from cv2utils.frameprocessor import FrameProcessor

class Tracker:

    ''' Create a new Tracker object with which to monitor video streams.

        Inputs:

        usb_dev : Video device number for usb camera (default=0 indicates '/dev/video0')
        vflip   : Boolean, indicates whether to vertically flip frames (default=False)
        hflip   : Boolean, indicates whether to horizontally flip frames (default=False)
        heartbeat_frames : Int, number of frames between heartbeat / FPS measurements (default=500)
    '''
    def __init__(self, usb_dev=0, vflip=False, hflip=False,
                 heartbeat_frames=500):

        self._fr_count = 0
        self._hb_time = 0
        self._heartbeat_frames = heartbeat_frames
        self.usb_dev = usb_dev
        self.vflip = vflip
        self.hflip = hflip
        self.subscribers = list()
        self._logger = Tracker._get_default_logger()
        print(__name__)

    def _update_subscribers(self, frame):
        for subscriber in self.subscribers:
            Thread(target=subscriber.update, args=(frame,)).start()
            #subscriber.update(frame)

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
        if (self.vflip and self.hflip):
            frame=cv2.flip(frame,-1)
        elif (self.vflip):
            frame=cv2.flip(frame,0)
        elif (self.hflip):
            frame=cv2.flip(frame,1)
        self._fr_count += 1
        self._update_subscribers(frame)
        self._heartbeat()

    '''
        run_usb - starts processing of video frames from usb camera
        Arguments:
          none

        Returns:
          none
    '''
    def run_usb(self):
        log = Tracker.get_logger()
        log.info("Started tracking on video" + str(self.usb_dev))
        # Initialize with one frame
        self._cap = cv2.VideoCapture(self.usb_dev)
        self._hb_time = time.time()
        self._stopped = False
        self._grabbed = False
        ret, frame = self._cap.read()
        log.info("Resolution = " + str(frame.shape))

        #while(True):
        #    ret, frame = self._cap.read()
        #    self._process_frame(frame)
        Thread(target=self._capture, args=()).start()
        return

    def _capture(self):
        while(True):
            if (self._stopped):
                return
            self._grabbed, frame = self._cap.read()
            self._process_frame(frame)

    def stop(self):
        # Stop the capture thread
        self._stopped = True

#    This works on raspberry pi, but not on Ubuntu because of dependencies
#    on pi camera object.  Need to figure out how to fix this, or put a derived
#    class in another package module so everything works on non-pi platforms.
#    def run(self):
#        log = Tracker.get_logger()
#        log.info("Started tracking on RaspiCam")
#        # Initialize with one frame
#        self._hb_time = time.time()
#        rawCapture = PiRGBArray(self._camera, size=self.res)
#        self._camera.capture(rawCapture, format="bgr")
#        log.info("Resolution = " + str(rawCapture.array.shape))
#
#        for frame in self._camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
#            self._process_frame(frame)
#            rawCapture.truncate(0)

    '''
        add_subscriber(self, subscriber)

        Description:
          Adds a subscriber object to the listener queue.  Each subscriber will receive frame
          updates for processing once the camera is running.
    '''
    def add_subscriber(self, subscriber):
        log = Tracker.get_logger()
        # Should add ID to events for logging clarity...
        log.info("Added subscriber [" + subscriber.name + "]")
        self.subscribers.append(subscriber)

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

#  Abstract handler object prototype.  Should be inherited for by any stateful
#  handler implementation object (although not explicitly type checked, duck typing
#  works just fine as long as handler object has a handle function with the
#  appropriate signature.
#  Should also add event detection metadata to handler signature, for convenience
#  and performance reasons
class Handler():

    #@abstractmethod
    def handle(contours, frame_buf, frame_index):
        raise NotImplementedError("Abstract class does not implement this method.")

#  Useful handler functions - TODO: update these to use new signature.

#  Print detection message to screen
#  This is now outdated since integrated logging outputs information automatically
#  for detected events, not necessary to put in a handler.  Non-standard detection
#  or state information could be logged from such a handler though.
def default_handler(contours, images, state=None):
    log = logging.getLogger(__name__)
    log.debug("Motion detected in " + str(len(contours)) + " regions!")
    contour, area = cvu.largest_contour(contours)
    log.debug("Largest centroid: " + str(cvu.centroid(contour)) + ", A=" + str(area))

#  Redo this to use new signature
#  Print detection message and save image showing detected motion contours
def save_image_handler(contours, frame_buf, frame_index):
    log = logging.getLogger(__name__)
    tmp = np.array(frame_buf[frame_index].shape) / 2
    imcenter = (int(tmp[1]), int(tmp[0]))
    buf_len = len(frame_buf)

    image = frame_buf[frame_index%buf_len]

    contour, area = cvu.largest_contour(contours)
    #centroid = cvu.centroid(contour)
    centroid = cvu.avg_contour_centroid(contours)

    log.debug("c = " + str(centroid))
    # Place the detected contours on the images
    cv2.drawContours(image, contours, -1,
                     color=(25,128,255), thickness=1)
    cvu.draw_x(image,centroid,length=9)
    cvu.draw_x(image,imcenter,length=9,thickness=1,shadow=False,color=(0xC0, 0xFF, 0x10))

    filename = cvu.imwrite_timestamp(image, "event_")
    log.info('Wrote ' + filename)


# This class takes in a set of contours (produced by upstream detection logic) and
# looks for specific conditions to be met:
#
# Area in largest detection contour > min_contour_area_px
# Area in largest detection contour < max_contour_area_px
# Time (s) since last triggered event > time_between_triggers_s
# Number of sequential frames meeting above criteria >= min_sequential_frames
class EventDetector:

    def __init__(self, time_between_triggers_s=1.0,
                 min_sequential_frames=1,
                 min_contour_area_px=500, max_contour_area_px=50000,
                 state=None):
        self._last_event_time = time.time()
        self._trigger_count = 0
        self.time_between_triggers_s = time_between_triggers_s
        self.min_contour_area_px = min_contour_area_px
        self.max_contour_area_px = max_contour_area_px
        self.min_sequential_frames = min_sequential_frames
        self._state = state

    def detection_ready(self):
        now = time.time()
        delta_t = now-self._last_event_time
        if (delta_t) < self.time_between_triggers_s :
            return False
        else:
            return True

    # Apply detection criteria and invoke handler if satisfied
    def detect(self, contours):
        if (contours is None): return False
        if len(contours) > 0 and self._meets_area_criteria(contours):
            self._trigger_count +=1
        else:
            self._trigger_count = 0
            return False
        if (self.detection_ready() is False):
            return False
        if self._trigger_count >= self.min_sequential_frames :
            now = time.time()
            delta_t = now - self._last_event_time
            self.event_metadata = EventMetadata(max_contour_area=self._largest_contour_area,
                                                n_seq_frames=self._trigger_count,
                                                time_between_triggers_s=delta_t,
                                                n_contours=len(contours))
            self._trigger_count = 0
            self._last_event_time = now
            return True

    def _meets_area_criteria(self, contours):
        contour, area = cvu.largest_contour(contours)
        self._largest_contour = contour
        self._largest_contour_area = area
        if area > self.min_contour_area_px and area < self.max_contour_area_px:
            return True
        return False

class EventMetadata():

    def __init__(self, max_contour_area, n_seq_frames,
                 time_between_triggers_s=0, n_contours=-1):
        self.max_contour_area = max_contour_area
        self.n_seq_frames = n_seq_frames
        self.time_between_triggers_s = time_between_triggers_s
        self.n_contours = n_contours

    def __str__(self):
        text = ("MaxArea = " + str(np.round(self.max_contour_area)) +
                ", nFrames = " + str(self.n_seq_frames) +
                ", dT = " + str(np.round(self.time_between_triggers_s,3)) +
                ", nContours = " + str(self.n_contours) )
        return text
