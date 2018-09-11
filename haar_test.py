import cv2
import numpy as np
from Webcam import Webcam

face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
wc = Webcam()
wc.start()
wc.fps_start()

while True:
    img = wc.read()

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    faces = face_cascade.detectMultiScale(gray, 1.3, 5)

    for x,y,w,h in faces:
        cv2.rectangle(img, (x,y), (x+w,y+h), (255,0,0), 2)
        roi_gray = gray[y:y+h, x:x+w]
        roi_color = img[y:y+h, x:x+w]
        #eyes = eye_cascade.detectMultiScale(roi_gray)


    cv2.imshow('img',img)

    wc.fps_update()
    if wc.num_frames == 60:
        wc.fps_stop()
        print(wc.fps())
        wc.fps_start()

    if cv2.waitKey(1) == ord('q'):
        break

wc.stop()
wc.stream.release()
cv2.destroyAllWindows()
