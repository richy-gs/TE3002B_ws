import time
from djitellopy import Tello

speed = 30

def main():
    me = Tello()
    me.connect()
    print("Battery:", me.get_battery())
    
    me.takeoff()
    time.sleep(1)
    me.send_rc_control(0,0,0,0)
    time.sleep(1)

    me.send_rc_control(speed,0,speed,0) # Izquierda y Arriba
    time.sleep(3)
    me.send_rc_control(0,0,0,0)
    time.sleep(1)

    me.send_rc_control(-speed,0,speed,0) # Derecha y Arriba
    time.sleep(3)
    me.send_rc_control(0,0,0,0)
    time.sleep(1)

    me.send_rc_control(-speed,0,-speed,0) # Derecha y Abajo
    time.sleep(3)
    me.send_rc_control(0,0,0,0)
    time.sleep(1)

    me.send_rc_control(speed,0,-speed,0) # Izquierda y Abajo
    time.sleep(3)
    me.send_rc_control(0,0,0,0)
    time.sleep(1)


    me.land()

if __name__ == '__main__':
    main()