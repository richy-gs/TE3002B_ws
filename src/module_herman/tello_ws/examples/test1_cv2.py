import cv2
import numpy as np


def drawPoints(img):
    cv2.circle(
        img, (50, 50), 5, (0, 0, 255), cv2.FILLED
    )  # Coordenadas adaptadas a imagen de 100x100


while True:
    img = np.zeros((1000, 1000, 3), np.uint8)
    drawPoints(img)
    cv2.imshow("Output", img)

    # Presiona 'q' para salir del bucle
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cv2.destroyAllWindows()
