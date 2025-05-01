import cv2
import numpy as np
from djitellopy import Tello

# Inicializar dron
tello = Tello()
tello.connect()
print("Battery:", tello.get_battery())
tello.streamon()

# Rango de color verde en HSV
lower = np.array([50, 100, 100])
upper = np.array([70, 255, 255])

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

            object_region = hsv[y:y+h, x:x+w]
            avg_color_rgb = cv2.mean(object_region)[:3]
            print(f"RGB color: R={avg_color_rgb[2]:.2f}, G={avg_color_rgb[1]:.2f}, B={avg_color_rgb[0]:.2f}")

            # Dibujo
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
            cv2.putText(frame, "Green detected", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        else:
            print("[INFO] Green not detected.")

        cv2.imshow("Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Interrupted by user.")

finally:
    tello.streamoff()
    cv2.destroyAllWindows()
