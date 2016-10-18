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

state=0

try:
    drone = libardrone.ARDrone()

    cap = cv2.VideoCapture('tcp://192.168.1.1:5555')

    drone.takeoff()

    detector = cv2.AKAZE_create()

    print ("Taking off...")
    drone.hover()

    for i in range(1,800):
        # Capture frame-by-frame
        ret, frame = cap.read()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #cv2.imwrite('01.png', gray)

        #Using AKAZE descriptors.

        #(kps, descs) = detector.detectAndCompute(gray, None)
        #print("keypoints: {}, descriptors: {}".format(len(kps), descs.shape))

        # draw the keypoints and show the output image
        #cv2.drawKeypoints(frame, kps, frame, (0, 255, 0))

        cv2.imshow("DroneView", gray)

        #print (drone.navdata.battery)

        state = i
        drone.set_speed(0.1)
        if (state == 200):
            print 'Forward'
            drone.move_forward()
        elif (state == 250):
            print 'Left'
            drone.hover()
            drone.turn_left()
        elif (state == 350):
            print 'Forward'
            drone.move_forward()
        elif (state == 380):
            print 'Left'
            drone.hover()
            drone.turn_left()
        elif (state == 420):
            print 'Forward'
            drone.move_forward()
        elif (state == 456):
            print 'Hover'
            drone.hover()

        k = cv2.waitKey(1)
        if k & 0xFF == ord('q'):
          break
        if (k == ord('a')):
          print("Move left")
          drone.move_left()
        if (k == ord('d')):
          print("Move right")
          drone.move_right()
        if (k == ord('w')):
          print("Move forward")
          drone.move_forward()
        if (k == ord('s')):
          print("Move backward")
          drone.move_backward()
        if (k == ord('c')):
          print('Turn Right')
          drone.turn_right()
        if (k == ord('z')):
          drone.turn_left()
        if (k == ord('x')):
          drone.hover()
          print("Hover")

    print ('Land Drone.')
    drone.land()

    print ('Landing...')
    time.sleep(10)

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
