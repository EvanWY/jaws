import numpy as np
import sys
import cv2

cap = cv2.VideoCapture(0)

while(cap.isOpened()):
    ret, frame = cap.read()
    if ret==True:        
        sys.stdout.buffer.write(frame.tobytes())
    else:
        break

cap.release()
