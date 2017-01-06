#!/usr/bin/python3

# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import sys

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

def get_contours(img, thresh=128, max=255, dilate=True, erode=True):
    
    th, dst = cv2.threshold(img, thresh, max, cv2.THRESH_BINARY)

    if (erode): dst = cv2.erode(dst, None, iterations=1)
    if (dilate): dst = cv2.dilate(dst, None, iterations=2)
    
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
    
def largest_contour(contours):
    max_area = 0
    largest = None
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > max_area :
            max_area = area
            largest = contour
    return contour, max_area 

def contour_area(contours):
    for contour in contours:
        area = cv2.contourArea(contour)
        print("A=" + str(area))

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
def get_motion_image(images, contours, axis=1, draw_contours=True, 
                     line_width=1, color=(0,0,255)):
    if len(contours) > 0 and draw_contours :
        for image in images :
            cv2.drawContours(image, contours, -1, color, line_width)
    img1 = images[0]; img2 = images[1]; diff = images[2]
    tmp = np.concatenate((img1, diff), axis=axis)
    result = np.concatenate((tmp, img2), axis=axis)
    return result
