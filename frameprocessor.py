#!/usr/bin/python3

#imports
import cv2
import time
import numpy as np
#import logging
import types
#from threading import Thread

import cv2utils.cv2utils as cvu

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

class ColorDetector(FrameProcessor):

    # Use bounded color box for detection criterion
    def __init__(self, bounds=([0, 200, 200],[179, 255, 255]),
                 use_hsv=True ):
        self.bounds = bounds
        self.lower = np.array(bounds[0], dtype="uint8")
        self.upper = np.array(bounds[1], dtype="uint8")
        self.use_hsv = use_hsv

        print("lower = " + str(self.lower))
        print("upper = " + str(self.upper))

    def process(self, frame_buf, frame_index):
        image = frame_buf[frame_index]
        if (image is None):
            return None, None
        if self.use_hsv:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(image, self.lower, self.upper)
        dst = cv2.bitwise_and(image, image, mask=mask)
        dst_gray = cv2.cvtColor(dst, cv2.COLOR_RGB2GRAY)
        #dst_gray = cv2.cvtColor( dst, cv2.COLOR_RGB2GRAY )

        contours, mask = cvu.get_contours(dst_gray, thresh=1,
                                      erode=True, dilate=True)
        #im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL,
        #                                            cv2.CHAIN_APPROX_SIMPLE)
        return contours, dst

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

