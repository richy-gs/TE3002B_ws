import cv2
import mediapipe as mp
from djitellopy import Tello
import time

# --- Configurar Tello ---
me = Tello()
me.connect()
print(f"Battery: {me.get_battery()}%")
me.streamon()
# me.takeoff()

# --- Configurar MediaPipe Hands ---
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
hands = mp_hands.Hands(
    max_num_hands=2,
    model_complexity=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.5
)

# Utilidad: decidir qué dedos están levantados
FINGER_TIPS = [4, 8, 12, 16, 20]
FINGER_PIPS = [3, 6, 10, 14, 18]

def get_finger_states(hand_landmarks):
    """
    Devuelve una lista de booleanos [pulgar, índice, medio, anular, meñique]
    True si ese dedo está extendido.
    """
    states = []
    lm = hand_landmarks.landmark
    # Pulgar: comparar coordenadas x (eje diferente)
    states.append(lm[FINGER_TIPS[0]].x < lm[FINGER_PIPS[0]].x)
    # Otros dedos: punta sobre pip en coordenadas de imagen (y más pequeño)
    for tip, pip in zip(FINGER_TIPS[1:], FINGER_PIPS[1:]):
        states.append(lm[tip].y < lm[pip].y)
    return states

# Mantener registro del último comando para evitar spam
last_command = None
last_time = time.time()
CMD_INTERVAL = 1.0  # segundos

try:
    while True:
        frame = me.get_frame_read().frame
        frame = cv2.resize(frame, (640, 480))
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(image)

        command = None

        if results.multi_hand_landmarks:
            for hand_lms, handedness in zip(results.multi_hand_landmarks,
                                            results.multi_handedness):
                label = handedness.classification[0].label  # "Left" or "Right"
                states = get_finger_states(hand_lms)
                count = sum(states)

                # Dibujar puntos de referencia
                mp_drawing.draw_landmarks(
                    frame, hand_lms, mp_hands.HAND_CONNECTIONS)

                # MANO IZQUIERDA: despegue / aterrizaje
                if label == "Left":
                    if count == 5:
                        command = "takeoff"
                    elif count == 0:
                        command = "land"

                # # MANO DERECHA: movimiento básico
                # elif label == "Right":
                #     # solo dedo índice levantado -> avanzar
                #     if states[1] and count == 1:
                #         command = ("rc", 0, 30, 0, 0)
                #     # índice + medio -> retroceder
                #     elif states[1] and states[2] and count == 2:
                #         command = ("rc", 0, -30, 0, 0)
                #     # anular + meñique -> girar a la izquierda
                #     elif states[3] and states[4] and count == 2:
                #         command = ("rc", 0, 0, 0, -30)
                #     # solo pulgar -> subir
                #     elif states[0] and count == 1:
                #         command = ("rc", 0, 0, 30, 0)
                #     # solo meñique -> bajar
                #     elif states[4] and count == 1:
                #         command = ("rc", 0, 0, -30, 0)
                #     else:
                #         # todas las demás posturas => mantenimiento
                #         command = ("rc", 0, 0, 0, 0)

                # salir del bucle una vez que hemos asignado un comando
                if command:
                    break

        # Enviar comando si es nuevo y ha pasado suficiente tiempo
        now = time.time()
        if command and (command != last_command) and (now - last_time > CMD_INTERVAL):
            if command == "takeoff":
                # me.takeoff()
                print("Takeoff")
            elif command == "land":
                # me.land()
                print("Land")
            elif isinstance(command, tuple) and command[0] == "rc":
                # desempaquetar velocidades rc: izquierda-derecha, adelante-atrás, arriba-abajo, guiñada
                _, lr, fb, ud, yaw = command
                # me.send_rc_control(lr, fb, ud, yaw)
                print(f"RC cmd LR:{lr} FB:{fb} UD:{ud} YAW:{yaw}")
            last_command = command
            last_time = now

        # Superponer texto del comando actual
        if last_command:
            cv2.putText(frame, str(last_command), (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow("Tello Hand Control", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    me.streamoff()
    cv2.destroyAllWindows()
    hands.close()
