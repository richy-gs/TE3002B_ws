import cv2
import time
import threading
from djitellopy import Tello

speed = 30
video_filename = "tello_flight.avi"

# Función para mostrar y grabar el video
def video_stream(me, stop_event):
    # Preparar el grabador de video
    frame_read = me.get_frame_read()
    width, height = 360, 240
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(video_filename, fourcc, 30.0, (width, height))

    while not stop_event.is_set():
        frame = frame_read.frame
        img = cv2.resize(frame, (width, height))
        out.write(img)  # Grabar frame
        cv2.imshow("Tello Video", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop_event.set()
            break

    out.release()
    cv2.destroyAllWindows()

def main():
    me = Tello()
    me.connect()
    print("Battery:", me.get_battery())

    me.streamon()

    stop_event = threading.Event()
    video_thread = threading.Thread(target=video_stream, args=(me, stop_event))
    video_thread.start()

    # Vuelo automatizado
    me.takeoff()
    time.sleep(1)
    me.send_rc_control(0, 0, 0, 0)
    time.sleep(1)

    me.send_rc_control(speed, 0, speed, 0)  # Izquierda y arriba
    time.sleep(3)
    me.send_rc_control(0, 0, 0, 0)
    time.sleep(1)

    me.send_rc_control(-speed, 0, speed, 0)  # Derecha y arriba
    time.sleep(3)
    me.send_rc_control(0, 0, 0, 0)
    time.sleep(1)

    me.send_rc_control(-speed, 0, -speed, 0)  # Derecha y abajo
    time.sleep(3)
    me.send_rc_control(0, 0, 0, 0)
    time.sleep(1)

    me.send_rc_control(speed, 0, -speed, 0)  # Izquierda y abajo
    time.sleep(3)
    me.send_rc_control(0, 0, 0, 0)
    time.sleep(1)

    me.land()
    print("Landing...")

    stop_event.set()
    video_thread.join()

if __name__ == '__main__':
    main()
