import libardrone
drone = libardrone.ARDrone()
import time
time.sleep(2)
drone.recover()
drone.halt()
quit()
