import numpy as np
import cv2
# from cv_bridge import CvBridge

# bridge = CvBridge()
# Load an color image in grayscale
# frame = cv2.imread('image.jpg',0)
#Load standard
frame = cv2.imread('sim2.png')
cv2.imshow('original', frame)

hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

#Sim
# lower_red = (0, 200, 15)
# upper_red = (0, 255, 255)

#Real Robot
lower_red = (0, 127, 50) #was 30
upper_red = (6, 255, 255)

red_mask = cv2.inRange(hsv, lower_red, upper_red)

edges = cv2.Canny(red_mask, 400, 1400)
cv2.imshow('Edge', edges)


##################
# Find contours: https://docs.opencv.org/master/d4/d73/tutorial_py_contours_begin.html
contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
# Draw contours:
cv2.drawContours(frame, contours, 0, (0, 255, 0), 2)

# Calculate image moments of the detected contour
M = cv2.moments(contours[0])

# Print center: https://www.pyimagesearch.com/2016/02/01/opencv-center-of-contour/
centreX = int(M["m10"] / M["m00"])
centreY = int(M["m01"] / M["m00"])
print("centre X : '{}'".format(centreX))
print("centre Y : '{}'".format(centreY))

# Draw a circle at centre
cv2.circle(frame, (round(M['m10'] / M['m00']), round(M['m01'] / M['m00'])), 5, (0, 255, 0), -1)

# Show image:
cv2.imshow("outline contour & centroid", frame)

cv2.imshow('image',red_mask)
cv2.waitKey(0)
cv2.destroyAllWindows()