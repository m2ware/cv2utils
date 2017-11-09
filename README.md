# cv2utils
Python extensions to OpenCV for motion detection and event handling framework

Tracker class continuously monitors video and detects configured motion events.  When 
events are detected, user-defined handlers are invoked to take relevant action.  The 
tracker can be configured with multiple events to detect.  Event configuration includes:

- Minimum motion contour area required to trigger event
- Number of sequential motion frames required to trigger event
- Minimum time between triggered events
- User-defined event handler to invoke when triggered
  
Handlers are passed a 3-tuple of images (frame1, frame2, absdiff) as well as the contour
coordinates for all detected motion regions.

Includes examples and numerous CV2 utility functions.

See the Wiki for information and instructions for use:

https://github.com/m2ware/cv2utils/wiki

[Originally written for raspberry pi CV/robotics project]
