# ============================
# Código para procesar un video guardado
# ============================

import cv2

# Nombre del archivo de video que se desea procesar
video_file = "balls.mp4"  # Ajusta este nombre según el archivo que tengas

# Se crea un objeto VideoCapture para leer el video
cap = cv2.VideoCapture(video_file)

if not cap.isOpened():
    print("No se pudo abrir el video")
    exit()

while True:
    # Se leen los fotogramas del video de uno en uno
    ret, frame = cap.read()

    # 'ret' indica si se pudo leer un fotograma exitosamente
    if not ret:
        print("Fin del video o no se pudo leer el fotograma")
        break

    # Convertir cada fotograma a escala de grises para un preprocesamiento sencillo
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Aplica desenfoque Gaussiano para reducir el ruido y facilitar la detección de bordes
    img_blur = cv2.GaussianBlur(gray, (3, 3), 0)

    # Aplicar el algoritmo de Canny para detección de bordes en cada fotograma
    edges = cv2.Canny(gray, 100, 200)

    # Mostrar el fotograma original y el resultado del Canny
    cv2.imshow("Fotograma Original", frame)
    cv2.imshow("Deteccion de Bordes - Canny", edges)

    # Espera 30 milisegundos entre fotogramas. Presiona 'q' para salir.
    if cv2.waitKey(30) & 0xFF == ord("q"):
        break

# Liberar el objeto VideoCapture y cerrar las ventanas
cap.release()
cv2.destroyAllWindows()
