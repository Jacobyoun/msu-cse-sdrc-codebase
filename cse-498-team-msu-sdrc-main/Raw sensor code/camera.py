import cv2
import time

#Define camera
Cam = 0

 #########################################  Camera initialization  ########################################
cv2.namedWindow('frame',cv2.WINDOW_NORMAL)

cap = cv2.VideoCapture(Cam, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,240)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 2) 
resx,resy=(240,320)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,240)

while 1:
    ret, frame = cap.read()	
    #frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow('frame', gray)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
