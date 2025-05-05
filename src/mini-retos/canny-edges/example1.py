import sys

import cv2

# Obtiene el nombre de la imagen desde los argumentos de línea de comandos
if len(sys.argv) != 2:
    print("Uso: python3 code.py nombre_imagen (sin extensión)")
    sys.exit(1)

imagen_nombre = sys.argv[1]

# Lee la imagen original desde el archivo proporcionado
img = cv2.imread(imagen_nombre)

# Verifica si la imagen fue cargada correctamente
if img is None:
    print(f"No se pudo cargar la imagen: {imagen_nombre}")
    sys.exit(1)

# Muestra la imagen original en una ventana
cv2.imshow("Original", img)
cv2.waitKey(0)

# Convierte la imagen original a escala de grises
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Aplica desenfoque Gaussiano para reducir el ruido y facilitar la detección de bordes
img_blur = cv2.GaussianBlur(img_gray, (3, 3), 0)

# Detección de bordes usando el operador Sobel en dirección X
sobelx = cv2.Sobel(src=img_blur, ddepth=cv2.CV_64F, dx=1, dy=0, ksize=5)

# Detección de bordes usando el operador Sobel en dirección Y
sobely = cv2.Sobel(src=img_blur, ddepth=cv2.CV_64F, dx=0, dy=1, ksize=5)

# Combinación de detección de bordes en direcciones X y Y usando Sobel
sobelxy = cv2.Sobel(src=img_blur, ddepth=cv2.CV_64F, dx=1, dy=1, ksize=5)

# Muestra las imágenes resultantes de Sobel X, Y y XY
cv2.imshow("Sobel X", sobelx)
cv2.waitKey(0)
cv2.imshow("Sobel Y", sobely)
cv2.waitKey(0)
cv2.imshow("Sobel X Y usando función Sobel()", sobelxy)
cv2.waitKey(0)

# Detección de bordes usando el algoritmo Canny
edges = cv2.Canny(image=img_blur, threshold1=100, threshold2=200)

# Muestra la imagen resultante después de aplicar Canny
cv2.imshow("Canny Edge Detection", edges)
cv2.waitKey(0)

# Cierra todas las ventanas abiertas
cv2.destroyAllWindows()
