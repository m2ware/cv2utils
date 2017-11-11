#!/usr/bin/python3

#imports
import cv2
import time
import numpy as np
import logging
import types

import cv2utils.cv2utils as cvu
from cv2utils.frameprocessor import FrameProcessor, MotionProcessor
from cv2utils.tracker import Tracker

'''
    class Subscriber

    Description:
      Subscriber objects are added to a listener queue managed by the Tracker object.
      Each subscriber can assign specific frame processing and event detection logic.
'''
class Subscriber:

    '''
        Initialize Subscriber object.

        Arguments:
          frame_processor : frame processor object.  Default=None uses default motion frame processor
          event_detector  : event detector object.  Default=None uses default event detector
          handler         : event handler object responds to detection events.  Default=None gives
                            dummy logger.
          frame_buf_size  : int, default=2.  Frame buffer size dictates how many past frames are
                            kept for processing / detection logic requiring more than 2.
    '''
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
        if (isinstance(handler, types.FunctionType)):
            self._handler = Subscriber.get_dummy_handler_obj(handler)
        else:
            self._handler = handler
        self._frame_buf_size = frame_buf_size
        self._frame_buf = [None] * self._frame_buf_size
        self._frame_index = 0
        self.name = name

    ''' update(self, frame)

        Description:
          Called by tracker object for each new video frame.
        Arguments:
          frame : CV2 image frame passed to subscriber
        Returns:
          none
    '''
    def update(self, frame):
        # Store the frame in the frame buffer
        self._frame_buf[self._frame_index] = frame
        self._frame_index = (self._frame_index + 1) % self._frame_buf_size

        if (self._event_detector.detection_ready() is False):
            return
        # Run image processing to identify regions of interest
        contours, detectFrame = self._frame_processor.process(frame_buf=self._frame_buf,
                                                              frame_index=self._frame_index)
        # If event detector is triggered by detection artifact, then run
        # the event handler.
        if (self._event_detector.detect(contours)):
            log = Tracker.get_logger()
            log.info('[' + self.name + '] event detected')
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


