import serial
from lewansoul_lx16a import *
from math import sin, cos, pi
import time

SERIAL_PORT = '/dev/tty.usbserial-1110'

controller = ServoController(
    serial.Serial(SERIAL_PORT, 115200, timeout=1),
)

servo1_id = 1
servo3_id = 3

# control servos directly
# controller.move(1, 300) # move servo ID=1 to position 100
# controller.move(2, 400) # move servo ID=1 to position 100

# or through proxy objects
try:
    servo1 = controller.servo(1)
    servo3 = controller.servo(3)
except ServoTimeout as e:
    print(f"Servo {e.ID} is not connected. Exiting...")
    exit()

servo1.move(135/.24, time=1000)
servo3.move(45/.24, time=1000)
# servo2.move(300)
# synchronous move of multiple servos
# servo1.move_prepare(300)
# servo2.move_prepare(600)
# controller.move_start()
# t = 0
# while True:
#     servo1.move((sin(t) * 120 + 120)/.24)
#     servo2.move((cos(t) * 120 + 120)/.24)
#     # servo1.move(sin(t) * 120 + 120)

#     time.sleep(0.05)
    # t += 0.1