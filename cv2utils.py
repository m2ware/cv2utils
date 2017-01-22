#!/usr/bin/python3

# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import sys
import logging
#from .tracker import Tracker

writeComposite = True

def imdiff(image1, image2) :
    diff = cv2.subtract(image2, image1)
    return(diff)

def imdiff_gray(image1, image2) : 
    diff = imdiff(image1, image2)
    return cv2.cvtColor( diff, cv2.COLOR_RGB2GRAY )

def hist_print(img, bins):
    counts,edges = np.histogram(img, bins=bins)
    print(edges)
    print(counts)
    
def avg_contour_centroid(contours):
    wavg_centroid = np.double((0.,0.))
    total_area = 0.;
    for contour in contours:
        area = cv2.contourArea(contour)
        center = centroid(contour)
        wavg_centroid += np.double(center)*area
        total_area += area
    wavg_centroid /= total_area
    return (np.int(wavg_centroid[0]), np.int(wavg_centroid[1]))


def get_contours(img, thresh=128, max=255, dilate=True, erode=True):
    
    th, dst = cv2.threshold(img, thresh, max, cv2.THRESH_BINARY)

    if (erode): dst = cv2.erode(dst, None, iterations=1)
    
    kernel = np.ones((5,5), np.uint8)
    if (dilate): dst = cv2.dilate(dst, kernel=kernel, iterations=2)
    
    #im2, contours, hierarchy = cv2.findContours(dst,cv2.RETR_TREE,
    #                                            cv2.CHAIN_APPROX_SIMPLE)
    im2, contours, hierarchy = cv2.findContours(dst,cv2.RETR_EXTERNAL,
                                                cv2.CHAIN_APPROX_SIMPLE)
    return contours, dst

def centroid(contour):
    M = cv2.moments(contour)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    return (cx, cy)

# draw_x draws an x to mark a spot on an image
# im is an image
# xy is a tuple (x,y) for the center
# length is the length from center (symmetric)
# color is a 3tuple or list holding RGB values,
# or a scalar if image is BW    
def draw_x(img, xy, length=5, color=(0xFF, 0x7F, 0x00), 
           thickness=2, shadow=True, shadow_color=(200, 200, 200) ):
    imsize = img.shape
    x1 = xy[0]-length; x2 = xy[0]+length
    y1 = xy[1]-length; y2 = xy[1]+length
    
    if (shadow):
        cv2.line(img, (x1,y1),(x2, y2), color=shadow_color,
                 thickness=thickness+2)
        cv2.line(img, (x1,y2),(x2, y1), color=shadow_color,
                 thickness=thickness+2)
        
    cv2.line(img, (x1,y1),(x2, y2), color=color, thickness=thickness)
    cv2.line(img, (x1,y2),(x2, y1), color=color, thickness=thickness)
    
def largest_contour(contours):
    max_area = 0
    largest = None
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > max_area :
            max_area = area
            largest = contour
    return largest, max_area 

def contour_area(contours):
    result = ""
    for contour in contours:
        area = cv2.contourArea(contour)
        result += ("A=" + str(area) + "\n")
        return(result)

def frame_diff(img1, img2, thresh=25, max=255):
    diff = cv2.absdiff(img1, img2)
    diff_gray = cv2.cvtColor(diff, cv2.COLOR_RGB2GRAY)
    return diff, diff_gray

def imwrite_timestamp(img, prefix="", 
                      format="%Y-%m-%d_%I_%M_%S", ext="jpeg"):
    filename = prefix + time.strftime(format) + "." + ext
    cv2.imwrite(filename, img)
    return filename

# Stictch together an image showing frame1, frame2, and delta with
# identified contours.  Images tuple / list should contain:
#
#    (img1, img2, diff)
def get_motion_image(images, contours, axis=1):
    img1 = images[0]; img2 = images[1]; diff = images[2]
    tmp = np.concatenate((img1, diff), axis=axis)
    result = np.concatenate((tmp, img2), axis=axis)
    return result

    #if len(contours) > 0 and draw_contours :
    #    for image in images :
    #        cv2.drawContours(image, contours, -1, color, line_width)


def get_logger(logname='logger', filename='logger.log',
               console_log_level=logging.NOTSET,
               file_log_level=logging.NOTSET):
    logging.basicConfig(format='%(asctime)s %(message)s',
                        datefmt='[%y-%m-%d %I:%M:%S]',
                        filename=filename,
                        filemode='w')
    console = logging.StreamHandler()
    console.setLevel(console_log_level)
    logging.getLogger('').addHandler(console)
    logger = logging.getLogger(logname)
    return logger
