# Test script which tracks a yellow ping pong ball using a basic color filter
# combined with cv2.findContour.

import cv2
import numpy as np
from collections import deque
from Webcam import Webcam

wc = Webcam()
wc.start()
wc.fps_start()

buffer = 60
pts = deque(maxlen=buffer)

colorLower = (0,180,0)
colorUpper = (40,255,250)
while True:
    img = wc.read()
    height, width, layers = img.shape
    img = cv2.resize(img, (int(width*0.4),int(height*0.4)))

    blurred = cv2.GaussianBlur(img, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, colorLower, colorUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    _,cnts,_ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    center = None

    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

		# only proceed if the radius meets a minimum size
        if radius > 10:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
            cv2.circle(img, (int(x), int(y)), int(radius),(0, 255, 255), 2)
            cv2.circle(img, center, 5, (0, 0, 255), -1)

    # update the points queue
    pts.appendleft(center)

    for i in range(1, len(pts)):
        # if either of the tracked points are None, ignore them
        if pts[i - 1] is None or pts[i] is None:
            continue

		# otherwise, compute the thickness of the line and draw the connecting lines
        thickness = int(np.sqrt(buffer / float(i + 1)) * 2.5)
        cv2.line(img, pts[i - 1], pts[i], (0, 0, 255), thickness)

    cv2.imshow('img', img)

    if cv2.waitKey(1) == ord('q'):
        break

wc.stream.release()
cv2.destroyAllWindows()
