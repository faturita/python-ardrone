#coding: latin-1
# DOES NOT WORK

import time,sys
import ps_drone

drone = ps_drone.Drone()
drone.startup()

drone.reset()
while (drone.getBattery()[0]==-1): time.sleep(0.1)
print "Battery:" + str(drone.getBattery()[0]) + "," + str(drone.getBattery()[1])
#drone.useDemoMode(True)
#drone.getNDpackage(["demo", "vision_detect"])

time.sleep(0.1)

CDC = drone.ConfigDataCount
drone.setConfigAllID()
drone.mp4Video()
drone.frontCam()
while CDC==drone.ConfigDataCount: time.sleep(0.001)
drone.startVideo()
#drone.showVideo()

IMC = drone.VideoImageCount
stop = False
ground = False
while drone.VideoImageCount == IMC:
    time.sleep(0.001)
    IMC = drone.VideoImageCount
    key = drone.getKey()
    if key == " ":
        if ground:
            ground = False
        else:
            ground = True
        drone.groundVideo(ground)
    elif key and key != " ": stop = True
