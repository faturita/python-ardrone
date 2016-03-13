import socket
import cv2
import json

import cv2
import urllib
import numpy as np

stream=urllib.urlopen('http://192.168.1.1:5555/')


# This does not work but it could work.

# It must implement H.264 data format.

host = '192.168.1.1'
port = 5555
size = 1024000000
cv2.namedWindow("preview")
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((host,port))
bytes=''
while True:
    bytes+=stream.read(1024)
    a = bytes.find('\xff\xd8')
    b = bytes.find('\xff\xd9')
    if a!=-1 and b!=-1 and a!=b:
        jpg = bytes[a:b+2]
        bytes= bytes[b+2:]

        if (len(jpg)>0):
            #nparr = np.fromstring(jpg, np.uint8)
            #img_np = cv2.imdecode(nparr, cv2.CV_LOAD_IMAGE_COLOR)


            #i = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8),cv2.CV_LOAD_IMAGE_COLOR)
            print ( len(jpg) )
            cv2.imshow('i',np.fromstring(jpg, dtype=np.uint8))
            if cv2.waitKey(1) == 27:
                exit(0)
while True:

    data = s.recv()
    #frame=json.loads(data)


    cv2.imshow("preview", data)

s.close()
