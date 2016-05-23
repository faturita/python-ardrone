# Use the libardrone library to send the recovery flag signal to the AR DRONE Parrot
import libardrone
drone = libardrone.ARDrone()
import time
time.sleep(2)
drone.recover()
drone.halt()
quit()
