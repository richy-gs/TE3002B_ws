import sys

import cv2

"""
CÓDIGO 3: PROCESAMIENTO DE VIDEO EN TIEMPO REAL DESDE LA CÁMARA CON BARRAS DESLIZANTES

Este programa utiliza la cámara web (generalmente índice 0) y lee los fotogramas en tiempo real.
Del mismo modo, se aplican un desenfoque Gaussiano y el detector de bordes Canny, con dos
trackbars para ajustar los umbrales. Se presiona 'x' para salir.
"""


def procesar_camara():
    # Crea el objeto de captura de video para la cámara web principal (generalmente índice 0)
    cap_cam = cv2.VideoCapture(1)

    # Verifica si la cámara se abre correctamente
    if not cap_cam.isOpened():
        print("No se pudo abrir la cámara web.")
        sys.exit(1)

    # Ventanas para mostrar el video de la cámara
    cv2.namedWindow("Camara con Canny")

    # Función callback vacía
    def nothing(x):
        pass

    # Crea trackbars para threshold1 y threshold2
    cv2.createTrackbar("Threshold1", "Camara con Canny", 100, 500, nothing)
    cv2.createTrackbar("Threshold2", "Camara con Canny", 200, 500, nothing)

    while True:
        # Lee frame a frame desde la cámara
        ret_cam, frame_cam = cap_cam.read()

        # Verifica si el fotograma se leyó correctamente
        if not ret_cam:
            print("No se pudo leer el fotograma de la cámara.")
            break

        # Convierte el fotograma a escala de grises
        gray_cam = cv2.cvtColor(frame_cam, cv2.COLOR_BGR2GRAY)

        # Aplica un desenfoque Gaussiano para reducir el ruido
        blur_cam = cv2.GaussianBlur(gray_cam, (3, 3), 0)

        # Lee los valores actuales de los trackbars
        t1 = cv2.getTrackbarPos("Threshold1", "Camara con Canny")
        t2 = cv2.getTrackbarPos("Threshold2", "Camara con Canny")

        # Aplica Canny para detectar bordes
        edges_cam = cv2.Canny(blur_cam, t1, t2)

        # Muestra el resultado en tiempo real
        cv2.imshow("Camara con Canny", edges_cam)

        # Espera 1 ms y si se presiona 'x', salir
        key = cv2.waitKey(1) & 0xFF
        if key == ord("x"):
            break

    # Libera la cámara y destruye las ventanas
    cap_cam.release()
    cv2.destroyAllWindows()


procesar_camara()
