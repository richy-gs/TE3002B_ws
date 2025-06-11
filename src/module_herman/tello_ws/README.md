# TE3002B_ws: módulo `tello_ws/projects`

Repositorio de ejemplos y proyectos finales para el módulo **Vehículos Aéreos No Tripulados** (TE3002B) — especial enfoque en DJI Tello TT.

---

## 📖 Descripción

Este workspace contiene varios scripts y recursos para controlar un dron DJI Tello Robomaster TT desde Python usando:

- **DJITelloPy** para comunicación y control de vuelo por Wi-Fi  
- **OpenCV-Python** para detección y seguimiento de rostro y de manos  

Los proyectos incluidos son:

1. **Seguimiento facial** (`faceTracking.py` y carpeta `faceTracking/`):  
   Implementación final de un “drone-cameraman” que mantiene el rostro centrado y a distancia constante (2–3 m).

2. **Detección de manos** (`handController.py` y carpeta `hand_Detection/`):  
   Scripts que detectan gestos de mano para enviar comandos de vuelo (avance, retroceso, giros).

3. **Control manual por teclado y captura de imágenes** (`keyBoardControl_camera.py`, `keyboardControlImageCapture.py`):  
   Teleoperación básica y captura de fotogramas para generación de dataset.

4. **Pruebas y utilidades** (`takeOff_test.py`, `mapping.py`):  
   Scripts de prueba de despegue/aterrizaje y mapeo de rangos de altura u orientaciones.

---

## 🗂️ Estructura de carpetas

```text
tello_ws/projects/
├── Resources/
│   └── haarcascade_frontalface_default.xml
├── classes/
│   └── (módulos auxiliares: PID, utilidades de visión, etc.)
├── faceTracking/
│   └── (implementación modular de seguimiento facial)
├── hand_Detection/
│   └── (ejemplos de detección de manos: Haar, MediaPipe, etc.)
├── faceTracking.py
├── handController.py
├── keyBoardControl_camera.py
├── keyboardControlImageCapture.py
├── mapping.py
└── takeOff_test.py
