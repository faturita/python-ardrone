#coding: latin-1
# Use the libardrone library to send the recovery flag signal to the AR DRONE Parrot
import libardrone
import multiprocessing
import os
import time

if __name__ == '__main__':
    multiprocessing.freeze_support()
    
    print('PID: {}'.format(os.getpid()))

    drone = libardrone.ARDrone()
    
    print('PID: {}'.format(os.getpid()))
    
    time.sleep(2)
    drone.recover()
    drone.halt()
    quit()
