import time

import cv2
import numpy as np
from djitellopy import Tello

# --- SETUP ---
me = Tello()
me.connect()
print(f"Battery: {me.get_battery()}%")
me.streamon()

# drone takeoff
me.takeoff()
# give it a moment to stabilize
me.send_rc_control(0, 0, 25, 0)
time.sleep(2)

# frame dimensions
w, h = 360, 240
# forward/backward distance thresholds
fbRange = [2200, 3600]
# PID gains: [P_yaw, D_yaw, P_vertical]
pid = [0.4, 0.4, 0.5]
# previous horizontal error, and last known horizontal direction
pError = 0
last_error = 0


def findFace(img):
    """Detect the largest face in the frame."""
    faceCascade = cv2.CascadeClassifier(
        "projects/Resources/haarcascade_frontalface_default.xml"
    )
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = faceCascade.detectMultiScale(gray, 1.2, 8)

    centers = []
    areas = []
    for x, y, fw, fh in faces:
        cv2.rectangle(img, (x, y), (x + fw, y + fh), (0, 0, 255), 2)
        cx, cy = x + fw // 2, y + fh // 2
        area = fw * fh
        cv2.circle(img, (cx, cy), 5, (0, 255, 0), cv2.FILLED)
        centers.append((cx, cy))
        areas.append(area)

    if areas:
        i = areas.index(max(areas))
        return img, [centers[i], areas[i]]
    else:
        # no face: return zeroed info
        return img, [[0, 0], 0]


def trackFace(info, w, h, pid, pError, last_error):
    """
    Compute velocities for yaw, forward/back, and up/down.
    Returns:
      fb      - forward/back velocity
      ud      - up/down velocity
      yaw_spd - yaw velocity
      error_x - current horizontal error
      found   - whether a face was detected
    """
    (cx, cy), area = info
    found = area != 0

    # horizontal error (for yaw control)
    error_x = cx - w // 2
    # vertical error (for up/down control)
    error_y = (cy - h // 2) * -1  # invert for upward motion

    # PID yaw: P + D
    yaw_spd = pid[0] * error_x + pid[1] * (error_x - pError)
    yaw_spd = int(np.clip(yaw_spd, -100, 100))

    # P-only vertical speed
    ud = int(np.clip(pid[2] * error_y, -100, 100))

    # forward/backward to keep distance
    fb = 0
    if found and area > fbRange[1]:
        fb = -20
    elif found and area < fbRange[0]:
        fb = 20

    if not found:
        # no face: override to search in last direction
        # if last_error >= 0, rotate positive yaw; else rotate negative yaw
        yaw_spd = 30 if last_error >= 0 else -30
        # freeze other motions while searching
        fb = 0
        ud = 0

    # send velocities: (left/right, forward/back, up/down, yaw)
    me.send_rc_control(0, fb, ud, yaw_spd)

    return fb, ud, yaw_spd, error_x, found


# --- MAIN LOOP ---
while True:
    img = me.get_frame_read().frame
    img = cv2.resize(img, (w, h))

    img, info = findFace(img)
    fb, ud, yaw_spd, error_x, found = trackFace(info, w, h, pid, pError, last_error)

    # update errors only when face is found
    if found:
        pError = error_x
        last_error = error_x

    # debug printout
    print(f"FB: {fb:>4}, UD: {ud:>4}, Yaw: {yaw_spd:>4},  ErrX: {error_x}")

    cv2.imshow("Tello Face Tracking", img)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        me.land()
        break
