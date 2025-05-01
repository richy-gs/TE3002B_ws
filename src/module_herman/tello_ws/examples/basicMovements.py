from time import sleep

from djitellopy import Tello

me = Tello()
me.connect()
print(me.get_battery())

me.takeoff()
me.send_rc_control(0, 50, 0, 0)
sleep(2)
me.send_rc_control(0, 0, 0, 0)
me.land()
