

import libardrone
import time
import numpy as np
import cv2
import sys
import socket
import json
import multiprocessing

dofly = False
drone = libardrone.ARDrone()

def main():
    # Initialize UDP Controller Server on port 7778
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_address = ('0.0.0.0', 7778)
    print('Starting up Controller Server on %s port %s' % server_address, file=sys.stderr)
    sock.bind(server_address)
    sock.setblocking(0)
    sock.settimeout(0.001)

    cap = cv2.VideoCapture('tcp://192.168.1.1:5555')

    print("Taking off...")
    #drone.takeoff()

    try:
        for i in range(1, 800):
            # Capture frame-by-frame
            ret, frame = cap.read()
            if not ret:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            #cv2.imwrite('01.png', gray)

            # Using AKAZE descriptors.
            detector = cv2.AKAZE_create()
            kps, descs = detector.detectAndCompute(gray, None)
            # print("keypoints: {}, descriptors: {}".format(len(kps), descs.shape))

            # draw the keypoints and show the output image
            cv2.drawKeypoints(frame, kps, frame, (0, 255, 0))
            drone.set_speed(0.1)

            cv2.imshow("DroneView", frame)

            try:
                data, address = sock.recvfrom(128)
                if data:
                    msg = json.loads(data.decode('utf-8'))
                    if msg.get('status') == 'L':
                        break
            except Exception as e:
                pass

            k = cv2.waitKey(1)
            if k == ord('q'):
                break
            if k == ord('a'):
                drone.move_left()
            if k == ord('d'):
                drone.move_right()
            if k == ord('w'):
                drone.move_forward()
            if k == ord('s'):
                drone.move_backward()
            if k == ord('x'):
                drone.hover()

        print('Land Drone.')
        drone.land()

        print('Landing...')
        time.sleep(8)

        print('Halting Drone...')
        drone.halt()
        print('Drone halted.')

        print('Ending...')
    except Exception as e:
        print("Error: {}".format(e), file=sys.stderr)
        print('Land Drone.')
        drone.land()

        print('Landing...')
        time.sleep(2)

        print('Sending the emergency state.')
        drone.reset()

        drone.halt()

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    multiprocessing.freeze_support()
    main()
