# ============================
# Código para procesar la señal en vivo de la cámara web
# ============================

import cv2

# Se crea un objeto VideoCapture con el índice de la cámara (generalmente 0)
cap_webcam = cv2.VideoCapture(1)

if not cap_webcam.isOpened():
    print("No se pudo acceder a la cámara.")
    exit()

while True:
    ret, frame = cap_webcam.read()

    if not ret:
        print("No se pudo obtener la imagen de la cámara.")
        break

    # Convertir la imagen de la cámara a escala de grises
    gray_cam = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Aplica desenfoque Gaussiano para reducir el ruido y facilitar la detección de bordes
    img_blur = cv2.GaussianBlur(gray_cam, (3, 3), 0)

    # Aplicar detección de bordes Canny
    edges_cam = cv2.Canny(gray_cam, 200, 400)

    # Mostrar la captura de la cámara y el resultado de la detección de bordes
    cv2.imshow("Camara en vivo", frame)
    cv2.imshow("Deteccion de Bordes - Canny (Camara)", edges_cam)

    # Espera 1 milisegundo entre fotogramas. Presiona 'q' para salir
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap_webcam.release()
cv2.destroyAllWindows()
