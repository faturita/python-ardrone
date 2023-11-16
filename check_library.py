import libardrone
import multiprocessing

if __name__ == '__main__':
    multiprocessing.freeze_support()

    drone = libardrone.ARDrone()
    navdataframe = drone.navdata

    print (navdataframe)


    drone.halt()
