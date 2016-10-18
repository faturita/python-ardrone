#coding: latin-1

#This works only with OpenCV 3
# source /usr/local/bin/virtualenvwrapper.sh
# workon cv
import cv2


#cap = cv2.VideoCapture(0)
#cap = cv2.VideoCapture('/Users/rramele/Documents/AppleStore.Subiendo.I.mov')
#cap = cv2.VideoCapture('tcp://192.168.1.1:5555')
cap = cv2.VideoCapture('tcp://192.168.0.3/cgi-bin/fwstream.cgi?FwModId=0&PortId=1&PauseTime=0&FwCgiVer=0x0001')
#cap = cv2.VideoCapture('rtsp://192.168.0.3/cam0_0')

print ("Connecting..")

for i in range(1,800):
   # Capture frame-by-frame
   ret, frame = cap.read()

   #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
   #cv2.imwrite('01.png', gray)

   #Using AKAZE descriptors.
   #detector = cv2.AKAZE_create()
   #(kps, descs) = detector.detectAndCompute(gray, None)
   #print("keypoints: {}, descriptors: {}".format(len(kps), descs.shape))

   # draw the keypoints and show the output image
   #cv2.drawKeypoints(frame, kps, frame, (0, 255, 0))

   cv2.imshow("DroneView", frame)


   if cv2.waitKey(1) & 0xFF == ord('q'):
      break

print ('Done.')

#When everything done, release the capture
cap.release()
cv2.destroyAllWindows()