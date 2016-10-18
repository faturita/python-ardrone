#coding: latin-1

# This program can be used to track the drone position using a video camera
# and drive the drone from the video camera.


#This works only with OpenCV 3
# source /usr/local/bin/virtualenvwrapper.sh
# workon cv
import libardrone
#>>> You may need to call drone.recover() and wait a few seconds to reset emergency.
#>>>

dofly = False

import time
import numpy as np
import cv2

import sys

import socket

import json

import math

# Initialize UDP Controller Server on port 10001
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address = ('0.0.0.0', 7778)
print >> sys.stderr, 'Starting up Controller Server on %s port %s', server_address
sock.bind(server_address)
sock.setblocking(0)

drone = libardrone.ARDrone()

# Capture the image from your computer's camera.
cap = cv2.VideoCapture(0)

# cap.set(3,640)
# cap.set(4,480)
#cap = cv2.VideoCapture('/Users/rramele/Documents/AppleStore.Subiendo.I.mov')
#cap = cv2.VideoCapture('tcp://192.168.1.1:5555')

#window = namedWindow("TheWindow",1)

# Ready !
drone.takeoff()

print ("Taking off...")

try:

    avg = 0
    for i in range(1,80):

        # Capture frame-by-frame
        ret, frame = cap.read()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #cv2.imwrite('01.png', gray)

        #Using AKAZE descriptors.
        detector = cv2.AKAZE_create()
        (kps, descs) = detector.detectAndCompute(gray, None)
        #print("keypoints: {}, descriptors: {}".format(len(kps), descs.shape))

        if (i > 1): #Doesn't work if i == 0
            # Gets Average of x position of keypoints
            sumKps = 0
            count = 0
            while (count < len(kps)):
                sumKps += kps[count].pt[0]
                count += 1
            newAvg = sumKps/count

            # If the difference between new average and last iteration average is greater than 5,
            # then we consider that there was movement in the image
            # TODO: Test this magic number with the drone
            movement = avg - newAvg
            if (math.fabs(movement) > 5):
                pass
                #print("Moved")
            else:
                pass
                #print("Not Moved")

            avg = newAvg


        # draw the keypoints and show the output image
        cv2.drawKeypoints(frame, kps, frame, (0, 255, 0))
        drone.set_speed(0.1)

        cv2.imshow("DroneView", frame)


        # data, address = sock.recvfrom(128)
        #
        # if (data):
        #     msg = json.loads(data)
        #     if (msg.status == 'L'):
        #         break

        #print (drone.navdata.battery)
        #cv2.imshow("Aux", 0)
        k = cv2.waitKey(1)
        print k
        if (k == ord('q')):
           break
        if (k == ord('a')):
            print("Move left")
            drone.move_left()
        if (k == ord('d')):
            print("Move rigt")
            drone.move_right()
        if (k == ord('w')):
            print("Move forward")
            drone.move_forward()
        if (k == ord('s')):
            print("Move backward")
            drone.move_backward()
        if (k == ord('x')):
            drone.hover()
            print("Hover")

    print ('Land Drone.')
    drone.land()

    print ('Landing...')
    time.sleep(8)

    print ('Halting Drone...')
    drone.halt()
    print ('Drone halted.')

    print ('Ending...')
except:
    print ('Land Drone.')
    drone.land()

    print ('Landing...')
    time.sleep(2)

    print ('Sending the emergency state.')
    drone.reset()

    drone.halt()

#When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
