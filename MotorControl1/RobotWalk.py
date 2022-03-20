from math import sin, cos, pi
from lx16a import *
import time
import numpy as np

class RobotWalk():
	def __init__(self):
		
		#RIGHT LEG
		self.servo1 = None #right thigh
		self.servo2 = None #right shin
	    self.servo3 = None #right ankle
		
		#LEFT LEG
	    self.servo4 = None #left thigh
	    self.servo5 = None #left shin
	    self.servo6 = None #left ankle

	 #    #RIGHT LEG STARTING POSITIONS
		# self.servo1_start_pos = 67.92 #right thigh
		# self.servo2_start_pos = 101.28 #right shin
		# self.servo3_start_pos = 54.96 #right ankle

		# #LEFT LEG STARTING POSITIONS
		# self.servo4_start_pos = 68.16 #left thigh
		# self.servo5_start_pos = 156.48 #left shin
		# self.servo6_start_pos = 60.72 #left ankle

		#RIGHT LEG STARTING POSITIONS
		self.servo1_start_pos = 73.92 #right thigh
		self.servo2_start_pos = 94.08 #right shin
		self.servo3_start_pos = 40.08 #right ankle

		#LEFT LEG STARTING POSITIONS
		self.servo4_start_pos = 64.56 #left thigh
		self.servo5_start_pos = 158.16 #left shin
		self.servo6_start_pos = 49.44 #left ankle

		# Servo 1 =  63.6
		# Servo 2 =  94.56
		# Servo 3 =  58.32
		# Servo 4 =  70.08
		# Servo 5 =  157.44
		# Servo 6 =  63.84

	def initialize(self):
		'''
		Function to initialize the motors to their corresponding ID. Basically makes sure all the motors are connected

		Throws an exception if a motor cannot be detected
		'''

		try:
			#RIGHT LEG
			self.servo1 = LX16A(1) #right thigh
			self.servo2 = LX16A(2) #right shin
		    self.servo3 = LX16A(3) #right ankle
			
			#LEFT LEG
		    self.servo4 = LX16A(4) #left thigh
		    self.servo5 = LX16A(5) #left shin
		    self.servo6 = LX16A(6) #left ankle

		except ServoTimeoutError as e:
			print(f"Servo {e.id_} is not connected. Exiting...")
    		quit()

    def calibrate(self):
    	'''
		Function to calibrate the robot and have it move to its starting position

		returns nothing
    	'''
    	self.servo1.move(self.servo1_start_pos, time=1000, wait_to_complete=False)
		self.servo2.move(self.servo2_start_pos, time=1000, wait_to_complete=False)
		self.servo3.move(self.servo3_start_pos, time=1000, wait_to_complete=True)

		self.servo4.move(self.servo4_start_pos, time=1000, wait_to_complete=False)
		self.servo5.move(self.servo5_start_pos, time=1000, wait_to_complete=False)
		self.servo6.move(self.servo6_start_pos, time=1000, wait_to_complete=True)

	def check_motors(self):
		'''
		Function to move the motors in a specific pattern to check that all motors are functioning and that we can control them

		returns nothin
		'''

		self.servo1.move(63.6, time=1000, wait_to_complete=False)
		self.servo2.move(94.56, time=1000, wait_to_complete=False)
		self.servo3.move(58.32, time=1000, wait_to_complete=True)

		self.servo4.move(70.08, time=1000, wait_to_complete=False)
		self.servo5.move(157.44, time=1000, wait_to_complete=False)
		self.servo6.move(63.82, time=1000, wait_to_complete=True)

    def walk(self):
    	pass

    def robot(self)
    	LX16A.initialize("/dev/tty.usbserial-1110")
    	self.initialize()
    	self.calibrate()
    	self.check_motors()

if __name__ == '__main__':
	robot = RobotWalk()
	robot.robot()


