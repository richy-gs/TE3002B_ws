import time
import cv2
import numpy as np
from djitellopy import Tello
from time import sleep
import classes.keyPressModule as kp

# Initialize keyboard module
kp.init()

# Initialize drone
me = Tello()
me.connect()
print("Battery:", me.get_battery())
me.streamon()

# Resolution and PID setup
w, h = 360, 240
# fbRange = [2000, 10000]
# pid = [0.1, 0.1, 0]
# pError = 0

# Face detection function
def findFace(img):
    faceCascade = cv2.CascadeClassifier("projects/Resources/haarcascade_frontalface_default.xml")
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = faceCascade.detectMultiScale(imgGray, 1.2, 8)

    myFaceListC = []
    myFaceListArea = []

    for x, y, w, h in faces:
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cx = x + w // 2
        cy = y + h // 2
        area = w * h
        cv2.circle(img, (cx, cy), 5, (0, 255, 0), cv2.FILLED)

        myFaceListC.append([cx, cy])
        myFaceListArea.append(area)

    if len(myFaceListArea):
        i = myFaceListArea.index(max(myFaceListArea))
        return img, [myFaceListC[i], myFaceListArea[i]]
    else:
        return img, [[0, 0], 0]

# Keyboard control
def getKeyboardInput():
    lr, fb, ud, yv = 0, 0, 0, 0
    speed = 50

    if kp.getKey("LEFT"):
        lr = -speed
    elif kp.getKey("RIGHT"):
        lr = speed

    if kp.getKey("UP"):
        fb = speed
    elif kp.getKey("DOWN"):
        fb = -speed

    if kp.getKey("w"):
        ud = speed
    elif kp.getKey("s"):
        ud = -speed

    if kp.getKey("a"):
        yv = speed
    elif kp.getKey("d"):
        yv = -speed

    if kp.getKey("q"):
        me.land()
        sleep(3)
    if kp.getKey("e"):
        me.takeoff()
        sleep(3)

    return [lr, fb, ud, yv]

# Main loop
while True:
    img = me.get_frame_read().frame
    # img = cv2.resize(img, (w, h))
    # img, info = findFace(img)

    vals = getKeyboardInput()
    me.send_rc_control(vals[0], vals[1], vals[2], vals[3])

    # print("Area:", info[1], "Center:", info[0])
    cv2.imshow("Drone Face Detection", img)
    if cv2.waitKey(1) & 0xFF == ord("z"):  # Press 'z' to quit
        me.land()
        break
