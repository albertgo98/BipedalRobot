from math import sin, cos, pi
from lx16a import *
import time as t
import numpy as np

LX16A.initialize("/dev/tty.usbserial-1110")

try:
    #INITIALIZE MOTOR INSTANCES

    #RIGHT LEG
    servo1 = LX16A(1) #right thigh
    # servo2 = LX16A(2) #right shin
    servo3 = LX16A(3) #right ankle

    #LEFT LEG
    # servo4 = LX16A(4) #left thigh
    # servo5 = LX16A(5) #left shin
    # servo6 = LX16A(6) #left ankle

#Exception handling in the event one of the motors is disconnected or not connected to begin with
except ServoTimeoutError as e:
    print(f"Servo {e.id_} is not connected. Exiting...")
    quit()

start = t.time()

wait = True

#Pause for 5 seconds before beginning sequence
while wait:
    if t.time()-start > 5:
        wait = False

#Set motors to a specific angle to begin with; specify that it should take half a second to get there
#Also specify that we want each of the commands to be completed before moving on to the next
servo1.move(45, time=500, wait_to_complete=True)
servo3.move(45, time=500, wait_to_complete=True)
# servo1.move(45, time=0)

#The following sequence makes the thigh motor move in sin curve oscillatory pattern
#makes the ankle motor move in cos curve oscillatory pattern
dt = 0
go = True
while go:
    servo1.move(sin(dt) * 45 + 90)
    servo3.move(cos(dt)* 45 + 90)
    # servo2.move(cos(t) * 60 + 60)
    # servo1.move(sin(t) * 120 + 120)

    t.sleep(0.05)
    dt += 0.1

    if dt > 4*pi:
        go = False

# servo1.move(45, time=0)
# servo1.move(45, time=1000, wait = True)
# print(servo1.get_physical_angle())


# servo2.move(cos(t) * 60 + 60)

# dt = np.linspace(0, 4*pi, 120)

# for i in range(len(dt)):
#     servo1.move(sin(dt[i]) * 45 + 90)
#     servo3.move(cos(dt[i])* 45 + 90)