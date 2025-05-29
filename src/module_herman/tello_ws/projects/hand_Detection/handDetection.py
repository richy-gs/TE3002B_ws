import cv2
import mediapipe as mp
from djitellopy import Tello

# --- Initialize Tello ---
me = Tello()
me.connect()
print(f"Battery: {me.get_battery()}%")
me.streamon()

# --- Initialize MediaPipe Hands ---
mp_hands = mp.solutions.hands
mp_draw  = mp.solutions.drawing_utils
hands    = mp_hands.Hands(
    max_num_hands=2,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.5
)

try:
    while True:
        # Grab frame from Tello
        frame = me.get_frame_read().frame
        frame = cv2.resize(frame, (640, 480))
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Process with MediaPipe
        results = hands.process(img_rgb)

        # Draw hand landmarks
        if results.multi_hand_landmarks:
            for hand_lms in results.multi_hand_landmarks:
                mp_draw.draw_landmarks(
                    frame, hand_lms, mp_hands.HAND_CONNECTIONS)

        # Show
        cv2.imshow("Tello Hand Detection Test", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    me.streamoff()
    hands.close()
    cv2.destroyAllWindows()
