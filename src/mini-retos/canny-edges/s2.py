import sys

import cv2

"""
CÓDIGO 2: PROCESAMIENTO DE UN VIDEO ALMACENADO CON BARRAS DESLIZANTES PARA THRESHOLD

Este programa recibe como argumento el nombre de un archivo de video (por ejemplo: video.mp4).
Se leen los fotogramas en un bucle, se aplica un desenfoque Gaussiano y luego el detector
Canny. Para ajustar los umbrales de Canny en tiempo real, se utilizan dos trackbars.
Se presiona 'x' para salir.
"""


def procesar_video():
    if len(sys.argv) < 2:
        print("Uso: python3 code.py video.mp4")
        sys.exit(1)

    # Obtiene el nombre del video desde los argumentos
    video_file = sys.argv[1]

    # Crea el objeto de captura de video con el archivo proporcionado
    cap = cv2.VideoCapture(video_file)

    # Verifica si el video se abrió correctamente
    if not cap.isOpened():
        print(f"No se pudo abrir el video: {video_file}")
        sys.exit(1)

    # Ventanas para mostrar el video
    cv2.namedWindow("Video con Canny")

    # Función callback vacía para las trackbars
    def nothing(x):
        pass

    # Crea los trackbars para threshold1 y threshold2
    cv2.createTrackbar("Threshold1", "Video con Canny", 100, 500, nothing)
    cv2.createTrackbar("Threshold2", "Video con Canny", 200, 500, nothing)

    while True:
        # Lee frame a frame
        ret, frame = cap.read()

        # Si no hay más fotogramas, salimos del bucle
        if not ret:
            print("Fin del video o error al leer.")
            break

        # Convierte el fotograma a escala de grises
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Aplica un desenfoque Gaussiano para reducir el ruido
        blur_frame = cv2.GaussianBlur(gray_frame, (3, 3), 0)

        # Obtiene los valores de los trackbars
        t1 = cv2.getTrackbarPos("Threshold1", "Video con Canny")
        t2 = cv2.getTrackbarPos("Threshold2", "Video con Canny")

        # Aplica el detector de bordes Canny
        edges_frame = cv2.Canny(blur_frame, t1, t2)

        # Muestra el fotograma resultante
        cv2.imshow("Video con Canny", edges_frame)

        # Espera 1 ms para mostrar el frame y captura la tecla presionada
        key = cv2.waitKey(1) & 0xFF
        if key == ord("x"):
            break

    # Libera el objeto de captura y cierra ventanas
    cap.release()
    cv2.destroyAllWindows()


procesar_video()
