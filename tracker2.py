#!/usr/bin/python3

#imports
import cv2
import time
import numpy as np
#from picamera.array import PiRGBArray
#from picamera import PiCamera
import logging
import types

import cv2utils.cv2utils as cvu

class Tracker:

    def __init__(self,
                 heartbeat_frames=500, usb_dev=0,
                 vflip=False, hflip=False):

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
            subscriber.update(frame)

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
        if (self.vflip):
            img=cv2.flip(img,0)
        if (self.hflip):
            img=cv2.flip(img,1)
        self._fr_count += 1
        self._update_subscribers(frame)
        self._heartbeat()

    def run_usb(self):
        log = Tracker.get_logger()
        log.info("Started tracking on video" + str(self.usb_dev))
        # Initialize with one frame
        cap = cv2.VideoCapture(self.usb_dev)
        self._hb_time = time.time()
        ret, frame = cap.read()
        log.info("Resolution = " + str(frame.shape))

        while(True):
            ret, frame = cap.read()
            self._process_frame(frame)

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

class Subscriber:

    def __init__(self, frame_processor=None,
                 event_detector=None,
                 handler=None, frame_buf_size=2,
                 name='subscriber1' ):

        if (frame_processor is None):
            # Default motion processor
            self._frame_processor=MotionProcessor()
        else :
            self._frame_processor = frame_processor
        if (event_detector is None):
            self._event_detector = EventDetector()
        else :
            self._event_detector = event_detector
        # If handler is a stateless function, wrap it in a dummy object
        if (isinstance(handler, types.FunctionType):
            self._handler = Subscriber.get_dummy_handler_obj(handler)
        else:
            self._handler = handler
        self._frame_buf_size = frame_buf_size
        self._frame_buf = [None] * self._frame_buf_size
        self._frame_index = 0
        self.name = name

    def update(self, frame):
        # Store the frame in the frame buffer
        self._frame_buf[self._frame_index] = frame
        self._frame_index = (self._frame_index + 1) % self._frame_buf_size
        # Run image processing to identify regions of interest
        contours, detectFrame = self._frame_processor.process(frame_buf=self._frame_buf,
                                                              frame_index=self._frame_index)
        # If event detector is triggered by detection artifact, then run
        # the event handler.
        if (self._event_detector.detect(contours)):
            log = Tracker.get_logger()
            log.info(self.name + ': event detected')
            log.debug(self._event_detector.event_metadata)
            if (self._handler is not None):
                self._handler.handle(contours, self._frame_buf, self._frame_index)

    # Create a dummy wrapper object for handler function not requiring any 
    # state information.
    @staticmethod
    def get_dummy_handler_obj(handler_fun):
        handler_obj = type('DummyHandler', (), {})
        # Really should check the function inputs to make sure it matches 
        # correct signature (contours, frame_buffer, frame_index)
        handler_obj.handle = handler_fun
        return handler_obj


class FrameProcessor():

    def __init__(self, threshold=25):
        self.threshold=threshold

    #@abstractmethod
    def process(frame_buf, frame_index):
        raise NotImplementedError("Abstract class does not implement this method.")

# Stateless motion detector - requires at least two frames, uses frame differencing
# to detect motion.
class MotionProcessor(FrameProcessor):

    def process(self, frame_buf, frame_index):
        n_frames = len(frame_buf)
        if (n_frames < 2):
            raise Exception("Motion detect requires at least 2 frames in frame buffer.")
        img1 = frame_buf[frame_index]
        img2 = frame_buf[(frame_index-1)%n_frames]
        if (img1 is None) or (img2 is None):
            return None, None
        diff, diff_gray = cvu.frame_diff(img1, img2)
        contours, mask = cvu.get_contours(diff_gray, thresh=self.threshold,
                                          erode=True, dilate=True)
        return contours, diff

class BrightInPlane(FrameProcessor):

    # Restrict to a single color plane, find bright spots
    def __init__(self, threshold=25, color_plane=0):
        self._color_plane = color_plane
        self.threshold=threshold

    #  This detector looks at only one color plane, removes the average across the image,
    #  and identifies regions exhibiting high values.
    def process(self, frame_buf, frame_index):

        image = frame_buf[frame_index]

        mean_removed_plane = cvu.mean_remove(image[:,:,self._color_plane])
        contours, mask = cvu.get_contours(mean_removed_plane, thresh=self._threshold,
                                          erode=True, dilate=True)
        return contours, image[:,:,self._color_plane]

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
def save_motion_image_handler(contours, images, state=None):
    log = logging.getLogger(__name__)
    tmp = np.array(images[0].shape) / 2
    imcenter = (int(tmp[1]), int(tmp[0]))

    default_handler(contours, images)
    contour, area = cvu.largest_contour(contours)
    #centroid = cvu.centroid(contour)
    centroid = cvu.avg_contour_centroid(contours)

    log.debug("c = " + str(centroid))
    # Place the detected contours on the images
    for image in images:
        cv2.drawContours(image, contours, -1,
                         color=(25,128,255), thickness=1)
        cvu.draw_x(image,centroid,length=9)
        cvu.draw_x(image,imcenter,length=9,thickness=1,shadow=False,color=(0xC0, 0xFF, 0x10))

    result = cvu.get_motion_image(images, contours)
    filename = cvu.imwrite_timestamp(result, prefix="Event_")
    log.info("Wrote " + filename)

# This class takes in a set of contours (produced by upstream detection logic) and
# looks for specific conditions to be met:
#
# Area in largest detection contour > min_contour_area_px
# Area in largest detection contour < max_contour_area_px
# Time (s) since last triggered event > time_between_triggers_s
# Number of sequential frames meeting above criteria >= min_sequential_frames
class EventDetector:

    def __init__(self, handler=default_handler,
                 time_between_triggers_s=1.0,
                 min_sequential_frames=1,
                 min_contour_area_px=500, max_contour_area_px=50000,
                 state=None):
        self.handler = handler
        self._last_event_time = time.time()
        self._trigger_count = 0
        self.time_between_triggers_s = time_between_triggers_s
        self.min_contour_area_px = min_contour_area_px
        self.max_contour_area_px = max_contour_area_px
        self.min_sequential_frames = min_sequential_frames
        self._state = state

    # Apply detection criteria and invoke handler if satisfied
    def detect(self, contours):
        if (contours is None): return False
        if len(contours) > 0 and self._meets_area_criteria(contours):
            self._trigger_count +=1
        else:
            self._trigger_count = 0
            return False
        now = time.time()
        delta_t = now-self._last_event_time
        if (delta_t) < self.time_between_triggers_s :
            return False
        if self._trigger_count >= self.min_sequential_frames :
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
