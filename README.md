This is a customized fork of the original library developed by git://github.com/venthur/python-ardrone.git

Getting Started:
----------------

```python
>>> import libardrone
>>> drone = libardrone.ARDrone()
>>> # You might need to call drone.reset() before taking off if the drone is in
>>> # emergency mode
>>> drone.takeoff()
>>> drone.land()
>>> drone.halt()
```

In the original library, the drone's property `image` contains always the latest image from the camera.  Now that code is defunct and the parsing of the frames is not working anymore.   Anyway, you can get the streaming directly by using OpenCV.

On the other hand, the drone's property `navdata` contains always the latest navdata.


DroneStreaming.py:
-----

This is the basic app that heavily uses the lib-ardrone library but get the streaming information from the OpenCV 3 VideoCapture function.


Requirements:
-------------

This software was tested with the following setup:

  * Python 2.7.4
  * OpenCV3.0.0
  * FFMPEG 3.1.4
  * Unmodified AR.Drone firmware 1.5.1

Compiling:
---------

 http://logdown.com/account/posts/386161-smart-flying

License:
--------

This software is published under the terms of the MIT License:

  http://www.opensource.org/licenses/mit-license.php
