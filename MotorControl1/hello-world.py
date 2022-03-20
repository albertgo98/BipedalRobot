from math import sin, cos, pi
from lx16a import *
import time as t
import numpy as np

LX16A.initialize("/dev/tty.usbserial-1110")

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

servo1_init_pos = servo1.get_physical_angle()
servo2_init_pos = servo2.get_physical_angle()
servo3_init_pos = servo3.get_physical_angle()
servo4_init_pos = servo4.get_physical_angle()
servo5_init_pos = servo5.get_physical_angle()
servo6_init_pos = servo6.get_physical_angle()

servo1.move(servo1_init_pos, time=50, wait_to_complete=False)
servo2.move(servo2_init_pos, time=50, wait_to_complete=False)
servo3.move(servo3_init_pos, time=50, wait_to_complete=True)


servo4.move(servo4_init_pos, time=50, wait_to_complete=False)
servo5.move(servo5_init_pos, time=50, wait_to_complete=False)
servo6.move(servo3_init_pos, time=50, wait_to_complete=True)

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


# servo4.move(25.28, time=750, wait_to_complete=False)
# servo5.move(86.0, time=750, wait_to_complete=False)
# servo6.move(63.84, time=750, wait_to_complete=True)

# Servo 1 =  63.12
# Servo 2 =  96.0
# Servo 3 =  58.8
# Servo 4 =  65.28
# Servo 5 =  148.8
# Servo 6 =  63.84

# time1 = int(((84.48-67.92)/120)*1000)
# time2 = int(((101.76-101.28/120))*1000)
# time3 = int(((54.96-40.8/120))*1000)

# servo1.move(63.6, time=750, wait_to_complete=False)
# servo2.move(60.56, time=750, wait_to_complete=False)
# servo3.move(58.32, time=750, wait_to_complete=True)

# servo1.move(73.92, time=750, wait_to_complete=False)
# servo2.move(94.08, time=750, wait_to_complete=False)
# servo3.move(40.08, time=750, wait_to_complete=False)

# servo4.move(64.56, time=750, wait_to_complete=False)
# servo5.move(158.16, time=750, wait_to_complete=False)
# servo6.move(49.44, time=750, wait_to_complete=True)

# servo4.move(70.08, time=750, wait_to_complete=False)
# servo5.move(157.44, time=750, wait_to_complete=False)
# servo6.move(63.82, time=750, wait_to_complete=True)

# servo3.move(40.08, time=750, wait_to_complete=False)
# servo2.move(94.08, time=750, wait_to_complete=False)
# servo1.move(73.92, time=750, wait_to_complete=False)

# print('RIGHT MOTOR INITIALIZED')

# servo4.move(63.84, time=750, wait_to_complete=False)
# servo5.move(157.2, time=750, wait_to_complete=False)
# servo6.move(54.24, time=750, wait_to_complete=True)

# print('LEFT MOTOR INITIALIZED')

# servo3.move(58.32, time=750, wait_to_complete=False)
# servo2.move(94.56, time=750, wait_to_complete=False)
# servo1.move(63.6, time=750, wait_to_complete=True)

# servo3.move(46.8, time=750, wait_to_complete=False)
# servo2.move(94.32, time=750, wait_to_complete=False)
# servo1.move(69.84, time=750, wait_to_complete=True)

# print('LEFT MOTOR INITIALIZED')

# servo1.move(63.6, time=750, wait_to_complete=False)
# servo3.move(58.32, time=750, wait_to_complete=False)
# servo2.move(94.56, time=750, wait_to_complete=True)


# print('RIGHT MOTOR RETURNED')

# servo4.move(70.08, time=750, wait_to_complete=False)
# servo5.move(157.44, time=750, wait_to_complete=False)
# servo6.move(63.82, time=750, wait_to_complete=True)
# print('LEFT MOTOR RETURNED')

# print('Servo 1 = ', servo1.get_physical_angle())
# print('Servo 2 = ', servo2.get_physical_angle())
# print('Servo 3 = ', servo3.get_physical_angle())
# print('Servo 4 = ', servo4.get_physical_angle())
# print('Servo 5 = ', servo5.get_physical_angle())
# print('Servo 6 = ', servo6.get_physical_angle())

# servo1.move(84.48, time=750, wait_to_complete=False)
# servo2.move(101.76, time=750, wait_to_complete=False)
# servo3.move(40.8, time=750, wait_to_complete=True)

# servo1.move(67.92, time=750, wait_to_complete=False)
# servo2.move(101.28, time=750, wait_to_complete=False)
# servo3.move(54.96, time=750, wait_to_complete=True)

# servo1.move(68.4, time=750, wait_to_complete=False)
# servo2.move(101.04, time=750, wait_to_complete=False)
# servo3.move(58.08, time=750, wait_to_complete=True)

###############################
###############################
###############################

# servo1.move(84.48, time=1000, wait_to_complete=True)
# servo2.move(101.76, time=1000, wait_to_complete=False)
# servo3.move(40.8, time=1000, wait_to_complete=True)

# print('Servo 1 = ', servo1.get_physical_angle())
# print('Servo 2 = ', servo2.get_physical_angle())
# print('Servo 3 = ', servo3.get_physical_angle())
# print('Servo 4 = ', servo4.get_physical_angle())
# print('Servo 5 = ', servo5.get_physical_angle())
# print('Servo 6 = ', servo6.get_physical_angle())

# initial_pos = servo1.get_physical_angle()
# diff = abs(111.72-initial_pos)
# servo1.move(66.72, time=1000, wait_to_complete=True)


# for i in range(45):
#     servo1.move(initial_pos-i)

#     t.sleep(0.05)





# servo1.move(45, time=0)

#The following makes the thigh motor move in sin curve oscillatory pattern makes the ankle motor move in cos curve oscillatory pattern
# dt = 0
# go = True
# while go:
#     servo1.move(sin(dt) * 45 + 90)
#     servo3.move(cos(dt)* 45 + 90)
#     # servo2.move(cos(t) * 60 + 60)
#     # servo1.move(sin(t) * 120 + 120)

#     t.sleep(0.05)
#     dt += 0.1

#     if dt > 4*pi:
#         go = False

# servo1.move(45, time=0)
# servo1.move(45, time=1000, wait = True)
# print(servo1.get_physical_angle())


# servo2.move(cos(t) * 60 + 60)

# dt = np.linspace(0, 4*pi, 120)

# for i in range(len(dt)):
#     servo1.move(sin(dt[i]) * 45 + 90)
#     servo3.move(cos(dt[i])* 45 + 90)