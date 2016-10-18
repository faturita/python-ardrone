#coding: latin-1

#This works only with OpenCV 3
# source /usr/local/bin/virtualenvwrapper.sh
# workon cv
import libardrone
#>>>
#>>> # You might need to call drone.reset() before taking off if the drone is in
#>>> # emergency mode
#>>>

dofly = False

import time
import numpy as np
import cv2

drone = libardrone.ARDrone()

#cap = cv2.VideoCapture(0)
#cap = cv2.VideoCapture('/Users/rramele/Documents/AppleStore.Subiendo.I.mov')
cap = cv2.VideoCapture('tcp://192.168.1.1:5555')
#cap = cv2.VideoCapture('output.avi')
#cap = cv2.VideoCapture('/Users/rramele/Desktop/Photos/DSC_0151.MOV')
#cap = cv2.VideoCapture('/Users/rramele/Downloads/centaur_2.mpg')
#window = namedWindow("TheWindow",1)

#drone.reset()

#drone.takeoff()

print ("Streaming...")

for i in range(1,2000):
   # Capture frame-by-frame
   ret, frame = cap.read()

   gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
   #cv2.imwrite('01.png', gray)

   #Using AKAZE descriptors.
   #detector = cv2.AKAZE_create()
   #(kps, descs) = detector.detectAndCompute(gray, None)
   #print("keypoints: {}, descriptors: {}".format(len(kps), descs.shape))

   # draw the keypoints and show the output image
   #cv2.drawKeypoints(frame, kps, frame, (0, 255, 0))

   cv2.imshow("DroneView", frame)

   navdataframe = drone.navdata[0]

   print navdataframe



   k = cv2.waitKey(1)
   if k & 0xFF == ord('q'):
      break
   if (k == ord('a')):
      print("Move left")


print ('Ending...')

drone.halt()

#When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
