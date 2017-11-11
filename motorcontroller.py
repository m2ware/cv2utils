#!/usr/bin/python3

from cv2utils.tracker import Tracker, Handler, EventDetector
from cv2utils.frameprocessor import MotionProcessor
from cv2utils.subscriber import Subscriber
import cv2utils.cv2utils as cvu
import os
import numpy as np
import logging

class MotorController(Handler):

    def __init__(self, h_pin=20, v_pin=21,
                 steering_gain=10.0,
                 bias_x=0, bias_y=0,
                 target_global_centroid=False):
        """
        Initialize MotorController

        Parameters:

        h_pin  - GPIO pin used for horizontal steering
        v_pin  - GPIO pin used for vertical steering
        steering_gain - Changes gain for camera steering sensitivity.  This
                        will vary depending on servos & camera view angle
        bias_x - Steering bias x offset, in pixels, relative to image center
        bias_y - Steering bias y offset, in pixels, relative to image center
        target_global_centroid - Boolean.  If false, will target center of the
                                 largest detection region.  If true, will
                                 target the area-weighted average of all
                                 detection regions.
        """

        # Servo position indicators, in increments of 100us pulse widths
        self.xpos = 15.0
        self.ypos = 15.0
        self.bias_x = bias_x
        self.bias_y = bias_y
        self.h_pin = h_pin
        self.v_pin = v_pin
        self.steering_gain = steering_gain
        self.target_global_centroid = target_global_centroid

    def handle(self, contours, frame_buf, frame_index):

        log = Tracker.get_logger()
        imsize = frame_buf[frame_index].shape
        hres=imsize[1]/2.
        vres=imsize[0]/2.

        tgtx = hres+self.bias_x
        tgty = vres+self.bias_y
        centroid = (0,0)

        # Designate target region
        if (self.target_global_centroid):
            centroid = cvu.avg_contour_centroid(contours)
        else:
            # Find area of largest motion contour
            contour, area = cvu.largest_contour(contours)
            centroid = cvu.centroid(contour)

        # Normalize the largest motion centroid in the [-1,1] space
        hpos = (centroid[0]-tgtx)/hres
        vpos = (centroid[1]-tgty)/vres
        # Convert to number of PWM pulses
        hdelta = self.steering_gain*(hpos*np.abs(hpos))
        vdelta = self.steering_gain*(vpos*np.abs(vpos))
        h_pulses = int(np.ceil(np.abs(hdelta)))+1
        v_pulses = int(np.ceil(np.abs(vdelta)))+1
        log.debug("hpc=" + str(h_pulses) + ", vpc=" + str(v_pulses))

        self.xpos -= hdelta;
        self.ypos += vdelta
        if (self.xpos > 22): self.xpos = 22
        if (self.xpos < 8): self.xpos = 8
        if (self.ypos > 22): self.ypos = 22
        if (self.ypos < 8): self.ypos = 8

        # These are hard-coded motor positions for L/R and U/D
        # 15 corresponds to 1500us position (neutral) on standard servo
        # Stuff for debug
        log.debug("N=" + str(len(contours)))
        #log.debug("A=" + str(area))
        log.debug("centroid = " + str(centroid))
        log.debug("hdelta=" + str(np.round(hdelta,2)) +
                ", vdelta=" + str(np.round(vdelta,2)))
        log.debug("htgt= " + str(np.round(self.xpos,2)) +
                ", vtgt= " + str(np.round(self.ypos,2)) )

        # Call command-line for PWM pulse train generator as detached processes
        # to steer incrementally toward detected motion.
        self.servo_steer(self.h_pin, h_pulses, self.xpos)
        self.servo_steer(self.v_pin, v_pulses, self.ypos)

    def servo_steer(self, pin, pulses, position):
        os.system("gpio_pwm " + str(pin) + " 10000 " + str(pulses) + " " +
                  str(position) + " &")
