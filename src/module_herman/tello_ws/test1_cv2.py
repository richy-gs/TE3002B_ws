# import pygame
# import numpy as np
# import cv2
# import sys

# # Inicializar pygame
# pygame.init()
# screen = pygame.display.set_mode((400, 300))
# pygame.display.set_caption("Ventana de prueba - Pygame")

# # Crear imagen negra con OpenCV
# img = np.zeros((300, 400, 3), dtype=np.uint8)
# cv2.putText(img, "Ventana de prueba - OpenCV", (10, 150),
#             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

# # Mostrar ventana de OpenCV
# cv2.imshow("Ventana OpenCV", img)

# running = True
# while running:
#     screen.fill((30, 30, 30))  # Pygame fondo gris oscuro

#     # Manejar eventos de Pygame
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT:
#             running = False

#     # Mostrar algo en la ventana de Pygame
#     font = pygame.font.SysFont(None, 32)
#     text = font.render('Ventana de prueba - Pygame', True, (255, 255, 255))
#     screen.blit(text, (50, 130))

#     pygame.display.flip()

#     # Manejar cierre desde OpenCV con tecla 'q'
#     if cv2.waitKey(10) & 0xFF == ord('q'):
#         running = False

# # Finalizar ventanas
# pygame.quit()
# cv2.destroyAllWindows()
# sys.exit()


import numpy as np
import cv2

def drawPoints(img):
    cv2.circle(img, (50, 50), 5, (0, 0, 255), cv2.FILLED)  # Coordenadas adaptadas a imagen de 100x100

while True:
    img = np.zeros((100, 100, 3), np.uint8)
    drawPoints(img)
    cv2.imshow("Output", img)
    
    # Presiona 'q' para salir del bucle
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
