#coding: latin-1
# Use the libardrone library to send the recovery flag signal to the AR DRONE Parrot
import libardrone
import multiprocessing

if __name__ == '__main__':
    multiprocessing.freeze_support()

    drone = libardrone.ARDrone()
    import time
    time.sleep(2)
    drone.land()
    time.sleep(0.1)
    drone.reset()
    quit()
