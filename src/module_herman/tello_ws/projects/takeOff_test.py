import time
from djitellopy import Tello

me = Tello()
me.connect()
print(me.get_battery())

me.takeoff()
me.send_rc_control(0, 0, 25, 0)
time.sleep(6)

me.send_rc_control(0, 0, 0, 0)
time.sleep(2)

me.land()
