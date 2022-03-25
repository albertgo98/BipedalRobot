from math import sin, cos, pi
from lx16a import *
import time as t
import numpy as np

LX16A.initialize("/dev/tty.usbserial-120")

try:
    #INITIALIZE MOTOR INSTANCES

    #RIGHT LEG
    servo1 = LX16A(1) #right thigh
    servo2 = LX16A(2) #right shin
    servo3 = LX16A(3) #right ankle

    #LEFT LEG
    servo4 = LX16A(4) #left thigh
    servo5 = LX16A(5) #left shin
    servo6 = LX16A(6) #left ankle

#Exception handling in the event one of the motors is disconnected or not connected to begin with
except ServoTimeoutError as e:
    print(f"Servo {e.id_} is not connected. Exiting...")
    quit()

start = t.time()

# wait = True

# #Pause for 5 seconds before beginning sequence
# while wait:
#     if t.time()-start > 2:
#         wait = False

# print('ready')
#Aim for 120 degrees per second

#Set motors to a specific angle to begin with; specify that it should take half a second to get there
#Also specify that we want each of the commands to be completed before moving on to the next

###############################
#####DEFAULT POSITIONS!!!!#####
###############################

# servo1.move(65.28, time=750, wait_to_complete=False)
# servo2.move(96.0, time=750, wait_to_complete=False)
# servo3.move(58.8, time=750, wait_to_complete=True)

# print('RIGHT MOTOR INITIALIZED')

# servo4.move(65.28, time=750, wait_to_complete=False)
# servo5.move(148.8, time=750, wait_to_complete=False)
# servo6.move(63.84, time=750, wait_to_complete=True)

# servo1.move(74.16, time=750, wait_to_complete=False)
# servo2.move(96.96, time=750, wait_to_complete=False)
# servo3.move(39.36, time=750, wait_to_complete=True)


# servo4.move(55.84, time=750, wait_to_complete=False)
# servo5.move(148.8, time=750, wait_to_complete=False)
# servo6.move(41.76-4.004, time=750, wait_to_complete=True)
print('LEFT MOTOR INITIALIZED')

print('Servo 1 = ', servo1.get_physical_angle())
print('Servo 2 = ', servo2.get_physical_angle())
print('Servo 3 = ', servo3.get_physical_angle())
print('Servo 4 = ', servo4.get_physical_angle())
print('Servo 5 = ', servo5.get_physical_angle())
print('Servo 6 = ', servo6.get_physical_angle())

# servo1_init_pos = servo1.get_physical_angle()
# servo2_init_pos = servo2.get_physical_angle()
# servo3_init_pos = servo3.get_physical_angle()
servo4_init_pos = servo4.get_physical_angle()
servo5_init_pos = servo5.get_physical_angle()
# servo6_init_pos = servo6.get_physical_angle()

# servo1.move(servo1_init_pos, time=50, wait_to_complete=False)
# servo2.move(servo2_init_pos, time=50, wait_to_complete=False)
# servo3.move(servo3_init_pos, time=50, wait_to_complete=True)


servo4.move(servo4_init_pos, time=500, wait_to_complete=False)
servo5.move(servo5_init_pos, time=50, wait_to_complete=False)
# servo6.move(servo6_init_pos, time=50, wait_to_complete=True)

# while True:
#     servo1.move(74.16, time=50, wait_to_complete=False)
#     servo2.move(96.96, time=50, wait_to_complete=False)
#     servo3.move(39.36, time=50, wait_to_complete=True)


#     servo4.move(49.68, time=50, wait_to_complete=False)
#     servo5.move(148.8, time=50, wait_to_complete=False)
#     servo6.move(41.76, time=50, wait_to_complete=True)

    # t.sleep(0.05)
#     start = t.time()

#     wait = True
#     while wait:
#         if t.time()-start > 0.05:
#             wait = False
