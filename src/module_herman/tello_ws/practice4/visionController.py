import cv2
import numpy as np
# import time
from djitellopy import Tello

# Inicializar dron
tello = Tello()
tello.connect()
print("Battery:", tello.get_battery())
tello.streamon()

# PD Parameters
Kp_yaw, Kd_yaw = 0.4, 0.2
Kp_z, Kd_z = 0.3, 0.1
prev_error_yaw = 0
prev_error_z = 0

# HSV para verde
lower = np.array([50, 100, 100])
upper = np.array([70, 255, 255])

# Centro de la imagen
frame_center_x = 480 // 2
frame_center_y = 360 // 2

# Takeoff
# tello.takeoff()
# time.sleep(2)

try:
    while True:
        frame = tello.get_frame_read().frame
        frame = cv2.resize(frame, (480, 360))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest)
            cx = x + w // 2
            cy = y + h // 2

            # PD control yaw
            error_yaw = cx - frame_center_x
            derivative_yaw = error_yaw - prev_error_yaw
            yaw_speed = int(Kp_yaw * error_yaw + Kd_yaw * derivative_yaw)
            yaw_speed = np.clip(yaw_speed, -90, 90)

            # PD control z
            error_z = frame_center_y - cy
            derivative_z = error_z - prev_error_z
            up_down_speed = int(Kp_z * error_z + Kd_z * derivative_z)
            up_down_speed = np.clip(up_down_speed, -20, 20)

            print("yawSpeed:", yaw_speed, "UpDownSpeed:", up_down_speed)

            # tello.send_rc_control(0, 0, up_down_speed, yaw_speed)

            prev_error_yaw = error_yaw
            prev_error_z = error_z

            # Visualización
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
            cv2.putText(frame, "Tracking green", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        else:
            # tello.send_rc_control(0, 0, 0, 0)
            print("No green detected.")

        cv2.imshow("Tracking", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Landing...")

finally:
    # tello.send_rc_control(0, 0, 0, 0)
    # time.sleep(1)
    # tello.land()
    tello.streamoff()
    cv2.destroyAllWindows()
