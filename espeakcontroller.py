#!/usr/bin/python3

from cv2utils.tracker2 import Tracker, Subscriber, Handler, MotionProcessor, EventDetector
import cv2utils.cv2utils as cvu
import os
import numpy as np
import logging
import random

class EspeakController(Handler):

    def __init__(self, flags='-v en+f4 -p 60 -s 190',
                 speech_items=['hello!', 'goodbye!'],
                 sequential=False):
        """
        Initialize EspeakController

        Parameters:

            flags - espeak command line flags for voice control
            speech_items - list of speech items
            sequential - if True, will loop through items.  If false, will randomize.
        """

        # Servo position indicators, in increments of 100us pulse widths
        self.flags = flags
        self._speech_items=speech_items
        self.sequential = sequential
        self._idx = 0;

    def handle(self, contours, frame_buf, frame_index):

        log = Tracker.get_logger()
        imsize = frame_buf[frame_index].shape
        idx = 0

        if self.sequential is True:
            idx = self._idx
            self._idx += 1
            self._idx = self._idx % len(self._speech_items)
        else:
            idx = random.randint(0, len(self._speech_items)-1)

        # Call espeak
        self._speak(idx)

    def _speak(self, idx=0):
        if len(self._speech_items) == 0:
            return
        os.system("espeak " + self.flags + " \"" +
                  self._speech_items[idx] + "\" --stdout | aplay &")
