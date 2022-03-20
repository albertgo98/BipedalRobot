from math import sin, cos, pi, sqrt, acos
from lx16a import *
import time
import numpy as np
from scipy.optimize import fsolve
import paho.mqtt.client as mqtt

class RobotController():
	'''
	Right and Left Thigh Motor Originally set to 65 degrees (at 65 degrees, the thigh faces directly up and down)
	Right Tendon Motor originally set to 135 degrees (at 135 degrees, the tendon faces directly up and down)
	Left Tendon Motor originally set to 125 degree (at 125 degrees, the tendon faces directly up and down)
	Right and Left Ankle Motor originally set to 65 degrees

	The right thigh motor and right tendon motor rotate CCW when increasing angle
	The left thigh motor and left tendon motor rotate CCW when increasing angle
	The right and left ankle motors rotate CCW when increasing angle

	The left tendon motor needs an additionall offset of 25.2 degrees
	'''
	def __init__(self):

		LX16A.initialize("/dev/tty.usbserial-1110")

		#INITIALIZE VARIABLES
		self.initialized = False
		self.calibrated = False
		self.left_motors_checked = False
		self.right_motors_checked = False
		self.final_calibration = False
		self.ready = False
		
		#RIGHT LEG MOTORS
		self.servo1 = None #right thigh
		self.servo2 = None #right shin
		self.servo3 = None #right ankle
		
		#LEFT LEG MOTORS
		self.servo4 = None #left thigh
		self.servo5 = None #left shin
		self.servo6 = None #left ankle

		#RIGHT LEG STARTING POSITIONS
		self.servo1_start_pos = 80.88 #right thigh
		self.servo2_start_pos = 96.48 #right shin
		self.servo3_start_pos = 34.32 #right ankle

		#LEFT LEG STARTING POSITIONS
		self.servo4_start_pos = 48.96 #left thigh
		self.servo5_start_pos = 148.8 #left shin
		self.servo6_start_pos = 34.8 #left ankle

		#LINKAGE LENGTHS IN INCHES
		self.l1 = 1.5 #distance between thigh and shin motors
		self.l2 = 6 #length of thigh
		self.l3 = 2 #length of tendon 1
		self.l4 = 6.84 #length of tendon 2
		self.l5 = 2 #distance between knee joint and shin joint
		self.l6 = 4 #distance between shin joint and ankle motor
		self.l7 = 2.09 #distance between ankle motor and bottom of foot

		##############################################
		#RIGHT LEG JOINT DEFINITIONS (FOR KINEMATICS)#
		##############################################
		self.A_x, self.A_y = 0, 0 #thigh motor joint
		self.B_x, self.B_y = 0, -self.l1 #tendon motor joint

		self.theta_1 = (pi/180)*(65.28-65) #thigh motor angle with the axis pointing up and down
		self.theta_2 = (pi/180)*(96.0-135) #tendon motor angle with the axis pointing up and down

		#Initial Knee Joint Position
		self.D_x = self.l2*cos(self.theta_1)
		self.D_y = self.l2*sin(self.theta_1)

		#Initial Tendon Joint Position
		self.C_x = self.l3*cos(self.theta_2)
		self.C_y = self.l3*sin(self.theta_2)-self.l1

		#Initial Shin Joint Position (ESTIMATION)
		self.E_x = self.D_x+(sqrt(4-0.1**2))
		self.E_y = self.D_y-0.1

		self.theta_4 = None
		if self.E_y < self.D_y:
			self.theta_4 = -acos((self.E_x-self.D_x)/self.l5)
		else:
			self.theta_4 = acos((self.E_x-self.D_x)/self.l5)

		print('Theta 4 = ', self.theta_4*(180/pi))

		self.theta_3 = cos((self.E_x-self.C_x)/self.l4)

		self.theta_6 = None

		if self.theta_4*(180/pi) < 0:
			self.theta_6 = pi/2-abs(self.theta_4)
		else:
			self.theta_6 = pi/2+self.theta_4

		print('Theta 3 = ', self.theta_3*(180/pi))
		print('Theta 6 = ', self.theta_6*(180/pi))

		#Initial Ankle Joint Position
		self.F_x = self.E_x+self.l6*cos(self.theta_4)
		self.F_y = self.E_y+self.l6*sin(self.theta_4)
		#############################################

		#############################################
		#LEFT LEG JOINT DEFINITIONS (FOR KINEMATICS)#
		#############################################
		self.G_x, self.G_y = 0, 0 #thigh motor joint
		self.H_x, self.H_y = 0, -self.l1 #tendon motor joint

		self.theta_7 = (pi/180)*(65.28-65) #thigh motor angle with the axis pointing up and down
		self.theta_8 = (pi/180)*(96.0-135) #tendon motor angle with the axis pointing up and down

		#Initial Knee Joint Position
		self.J_x = self.l2*cos(self.theta_7)
		self.J_y = self.l2*sin(self.theta_7)

		#Initial Tendon Joint Position
		self.I_x = self.l3*cos(self.theta_8)
		self.I_y = self.l3*sin(self.theta_8)-self.l1

		#Initial Shin Joint Position (ESTIMATION)
		self.K_x = self.D_x+(sqrt(4-0.1**2))
		self.K_y = self.D_y-0.1

		self.theta_10 = None
		if self.K_y < self.J_y:
			self.theta_10 = -acos((self.K_x-self.J_x)/self.l5)
		else:
			self.theta_10 = acos((self.K_x-self.J_x)/self.l5)

		print('Theta 10 = ', self.theta_10*(180/pi))

		self.theta_9 = cos((self.K_x-self.I_x)/self.l4)

		self.theta_12 = None

		if self.theta_10*(180/pi) < 0:
			self.theta_12 = pi/2-abs(self.theta_10)
		else:
			self.theta_12 = pi/2+self.theta_10

		print('Theta 9 = ', self.theta_9*(180/pi))
		print('Theta 12 = ', self.theta_12*(180/pi))

		#Initial Ankle Joint Position
		self.L_x = self.K_x+self.l6*cos(self.theta_10)
		self.L_y = self.K_y+self.l6*sin(self.theta_10)
		#############################################

		############################
		###SETTING UP MQTT CLIENT###
		############################

		#Define client name
		self.client_name = 'Rigi'
		self.mqtt_client = mqtt.Client(self.client_name)

		# Callback function is called when script is connected to mqtt server
		self.mqtt_client.on_connect = self.on_connect

        # Establishing and calling callback functions for when messages from different topics are received
		self.mqtt_client.message_callback_add(self.client_name + '/calibrate', self.on_message_calibrate)
		self.mqtt_client.message_callback_add(self.client_name + '/check_left', self.on_message_left)
		self.mqtt_client.message_callback_add(self.client_name + '/check_right', self.on_message_right)
		self.mqtt_client.message_callback_add(self.client_name + '/walk', self.on_message_walk)
		self.mqtt_client.on_message = self.on_message

        # Subscribe to everything that starts with Client name
		self.mqtt_client.connect("test.mosquitto.org", 1883, 60)
		self.mqtt_client.subscribe(self.client_name+'/#')

		if not self.initialized:
			self.initialize()
			self.initialized = True
			print('DONE INITIALIZING')
			self.mqtt_client.publish(self.client_name + "/Update", 'Done Initializing')


		self.mqtt_client.loop_forever()

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

		print('CALIBRATING...')

		self.servo1.move(self.servo1_start_pos, time=750, wait_to_complete=False)
		self.servo2.move(self.servo2_start_pos, time=750, wait_to_complete=False)
		self.servo3.move(self.servo3_start_pos, time=750, wait_to_complete=True)

		self.servo4.move(self.servo4_start_pos, time=750, wait_to_complete=False)
		self.servo5.move(self.servo5_start_pos, time=750, wait_to_complete=False)
		self.servo6.move(self.servo6_start_pos, time=750, wait_to_complete=True)

		#######################
		##CALIBRATE RIGHT LEG##
		#######################

		servo1_pos = self.servo1.get_physical_angle()
		servo2_pos = self.servo2.get_physical_angle()

		self.theta_1 = (pi/180)*(servo1_pos-65) #thigh motor angle with the axis pointing up and down
		self.theta_2 = (pi/180)*(servo2_pos-135) #tendon motor angle with the axis pointing up and down

		print('Theta 1 = ', self.theta_1)
		print('Theta 2 = ', self.theta_2)

		self.D_x = self.l2*cos(self.theta_1)
		self.D_y = self.l2*sin(self.theta_1)

		self.C_x = self.l3*cos(self.theta_2)
		self.C_y = self.l3*sin(self.theta_2)-self.l1

		self.E_x, self.E_y = fsolve(self.equations, (self.E_x, self.E_y), args=(self.C_x, self.C_y, self.D_x, self.D_y))

		self.theta_4 = None
		if self.E_y < self.D_y:
			self.theta_4 = -acos((self.E_x-self.D_x)/self.l5)
		else:
			self.theta_4 = acos((self.E_x-self.D_x)/self.l5)

		print('Theta 4 = ', self.theta_4*(180/pi))

		self.theta_3 = cos((self.E_x-self.C_x)/self.l4)

		self.theta_6 = None

		if self.theta_4*(180/pi) < 0:
			self.theta_6 = pi/2-abs(self.theta_4)
		else:
			self.theta_6 = pi/2+self.theta_4

		print('Theta 3 = ', self.theta_3*(180/pi))
		print('Theta 6 = ', self.theta_6*(180/pi))

		#Initial Ankle Joint Position
		self.F_x = self.E_x+self.l6*cos(self.theta_4)
		self.F_y = self.E_y+self.l6*sin(self.theta_4)

		######################
		##CALIBRATE LEFT LEG##
		######################

		servo4_pos = self.servo4.get_physical_angle()
		servo5_pos = self.servo5.get_physical_angle()

		self.theta_7 = (pi/180)*(65-servo4_pos) #thigh motor angle with the axis pointing up and down
		self.theta_8 = (pi/180)*(125-servo5_pos-15) #tendon motor angle with the axis pointing up and down

		print('Theta 7 = ', self.theta_7)
		print('Theta 8 = ', self.theta_8)

		self.J_x = self.l2*cos(self.theta_7)
		self.J_y = self.l2*sin(self.theta_7)

		self.I_x = self.l3*cos(self.theta_8)
		self.I_y = self.l3*sin(self.theta_8)-self.l1

		self.K_x, self.K_y = fsolve(self.equations, (self.K_x, self.K_y), args=(self.I_x, self.I_y, self.J_x, self.J_y))

		self.theta_10 = None
		if self.K_y < self.J_y:
			self.theta_10 = -acos((self.K_x-self.J_x)/self.l5)
		else:
			self.theta_10 = acos((self.K_x-self.J_x)/self.l5)

		print('Theta 10 = ', self.theta_10*(180/pi))

		self.theta_9 = cos((self.K_x-self.I_x)/self.l4)

		self.theta_12 = None

		if self.theta_10*(180/pi) < 0:
			self.theta_12 = pi/2-abs(self.theta_10)
		else:
			self.theta_12 = pi/2+self.theta_10

		print('Theta 9 = ', self.theta_9*(180/pi))
		print('Theta 12 = ', self.theta_12*(180/pi))

		#Initial Ankle Joint Position
		self.L_x = self.K_x+self.l6*cos(self.theta_10)
		self.L_y = self.K_y+self.l6*sin(self.theta_10)

	def check_both_motors(self):
		'''
		Function to move the motors in a specific pattern to check that all motors are functioning and that we can control them

		returns nothin
		'''
		print('CHECKING MOTORS...')

		servo1_init_pos = self.servo1.get_physical_angle()
		servo2_init_pos = self.servo2.get_physical_angle()
		servo3_init_pos = self.servo3.get_physical_angle()
		servo4_init_pos = self.servo4.get_physical_angle()
		servo5_init_pos = self.servo5.get_physical_angle()
		servo6_init_pos = self.servo6.get_physical_angle()

		servo1_goal_pos = 65.28
		servo2_goal_pos = servo2_init_pos
		# servo3_goal_pos = self.find_ankle_angle(servo1_goal_pos, servo2_goal_pos, servo3_init_pos)
		# print(servo3_goal_pos)
		servo4_goal_pos = 63.0 #64.72
		servo5_goal_pos = servo5_init_pos
		# servo6_goal_pos = servo6_init_pos

		diff1 = servo1_goal_pos-servo1_init_pos
		diff2 = servo2_goal_pos-servo2_init_pos
		# diff3 = servo3_goal_pos-servo3_init_pos
		diff4 = servo4_goal_pos-servo4_init_pos
		diff5 = servo5_goal_pos-servo5_init_pos
		# diff6 = servo6_goal_pos-servo6_init_pos

		dt = 0
		servo3_prev = servo3_init_pos
		servo6_prev = servo6_init_pos
		while dt < pi:

			print('MOVING TO NEW POSITION')

			servo1_next = sin(dt) * diff1 + servo1_init_pos
			servo2_next = sin(dt) * diff2 + servo2_init_pos
			servo3_next = self.find_ankle_angle(servo1_next, servo2_next, servo3_prev)
			print('Right Ankle Motor =', servo3_next)

			servo4_next = sin(dt) * diff4 + servo4_init_pos
			servo5_next = sin(dt) * diff5 + servo5_init_pos
			servo6_next = self.find_left_ankle_angle(servo4_next, servo5_next, servo6_prev)
			print('Left Ankle Motor =', servo6_next)

			self.servo1.move(servo1_next)
			self.servo2.move(servo2_next)
			self.servo3.move(servo3_next)

			self.servo4.move(servo4_next)
			self.servo5.move(servo5_next)
			self.servo6.move(servo6_next)

			time.sleep(0.05)
			dt += 0.07
			servo3_prev = servo3_next
			servo6_prev = servo6_next

	def check_right_motors(self):
		'''
		Function to move the motors in a specific pattern to check that all motors are functioning and that we can control them

		returns nothin
		'''
		print('CHECKING RIGHT MOTORS...')

		servo1_init_pos = self.servo1.get_physical_angle()
		servo2_init_pos = self.servo2.get_physical_angle()
		servo3_init_pos = self.servo3.get_physical_angle()

		servo1_goal_pos = 70.0
		servo2_goal_pos = servo2_init_pos

		diff1 = servo1_goal_pos-servo1_init_pos
		diff2 = servo2_goal_pos-servo2_init_pos

		dt = 0
		servo3_prev = servo3_init_pos
		while dt < pi:

			print('MOVING TO NEW POSITION')

			servo1_next = sin(dt) * diff1 + servo1_init_pos
			servo2_next = sin(dt) * diff2 + servo2_init_pos
			servo3_next = self.find_ankle_angle(servo1_next, servo2_next, servo3_prev)
			print('Right Ankle Motor =', servo3_next)

			self.servo1.move(servo1_next)
			self.servo2.move(servo2_next)
			self.servo3.move(servo3_next)

			time.sleep(0.05)
			dt += 0.07
			servo3_prev = servo3_next

	def check_left_motors(self):
		'''
		Function to move the motors in a specific pattern to check that all motors are functioning and that we can control them

		returns nothin
		'''
		print('CHECKING LEFT MOTORS...')

		servo4_init_pos = self.servo4.get_physical_angle()
		servo5_init_pos = self.servo5.get_physical_angle()
		servo6_init_pos = self.servo6.get_physical_angle()


		servo4_goal_pos = 63.0 #64.72
		servo5_goal_pos = servo5_init_pos

		diff4 = servo4_goal_pos-servo4_init_pos
		diff5 = servo5_goal_pos-servo5_init_pos

		dt = 0
		servo6_prev = servo6_init_pos
		while dt < pi:

			print('MOVING TO NEW POSITION')

			servo4_next = sin(dt) * diff4 + servo4_init_pos
			servo5_next = sin(dt) * diff5 + servo5_init_pos
			servo6_next = self.find_left_ankle_angle(servo4_next, servo5_next, servo6_prev)
			print('Left Ankle Motor =', servo6_next)

			self.servo4.move(servo4_next)
			self.servo5.move(servo5_next)
			self.servo6.move(servo6_next)

			time.sleep(0.05)
			dt += 0.1
			servo6_prev = servo6_next

	def find_ankle_angle(self, servo1_goal, servo2_goal, servo3_init):
		'''
		Function that finds that ankel motor goal given the knee position 

		knee_pos: list containing the coordinates of the knee position

		returns angle in degrees
		'''
		print('FINDING RIGHT ANKLE MOTOR')

		self.theta_1 = (pi/180)*(servo1_goal-65)
		self.theta_2 = (pi/180)*(servo2_goal-135)

		print('Theta 1 = ', self.theta_1)
		print('Theta 2 = ', self.theta_2)

		self.C_x = self.l3*cos(self.theta_2)
		self.C_y = self.l3*sin(self.theta_2)-self.l1

		self.D_x = self.l2*cos(self.theta_1)
		self.D_y = self.l2*sin(self.theta_1)

		self.E_x, self.E_y = fsolve(self.equations, (self.E_x, self.E_y), args=(self.C_x, self.C_y, self.D_x, self.D_y))

		self.theta_4 = None
		if self.E_y < self.D_y:
			self.theta_4 = -acos((self.E_x-self.D_x)/self.l5)
		else:
			self.theta_4 = acos((self.E_x-self.D_x)/self.l5)

		self.theta_3 = cos((self.E_x-self.C_x)/self.l4)

		new_theta_6 = None
		if self.theta_4*(180/pi) < 0:
			new_theta_6 = pi/2-abs(self.theta_4)
		else:
			new_theta_6 = pi/2+self.theta_4

		print('New Theta 6 = ', new_theta_6*(180/pi))

		theta_6_adj = self.theta_6-new_theta_6
		print('Theta Adjust =', theta_6_adj*(180/pi))
		self.theta_6 = new_theta_6
		goal = servo3_init-(theta_6_adj*(180/pi))

		return goal

	def find_left_ankle_angle(self, servo4_goal, servo5_goal, servo6_init):
		'''
		Function that finds that ankel motor goal given the knee position 

		knee_pos: list containing the coordinates of the knee position

		returns angle in degrees
		'''
		print('FINDING LEFT ANKLE MOTOR')

		self.theta_7 = (pi/180)*(65-servo4_goal)
		self.theta_8 = (pi/180)*(125-servo5_goal-15)

		print('Theta 7 = ', self.theta_7)
		print('Theta 8 = ', self.theta_8)

		self.I_x = self.l3*cos(self.theta_8)
		self.I_y = self.l3*sin(self.theta_8)-self.l1

		self.J_x = self.l2*cos(self.theta_7)
		self.J_y = self.l2*sin(self.theta_7)

		self.K_x, self.K_y = fsolve(self.equations, (self.K_x, self.K_y), args=(self.I_x, self.I_y, self.J_x, self.J_y))

		self.theta_10 = None
		if self.K_y < self.J_y:
			self.theta_10 = -acos((self.K_x-self.J_x)/self.l5)
		else:
			self.theta_10 = acos((self.K_x-self.J_x)/self.l5)

		print('Theta 10 = ', self.theta_10*(180/pi))

		self.theta_9 = cos((self.K_x-self.I_x)/self.l4)

		new_theta_12 = None
		if self.theta_10*(180/pi) < 0:
			new_theta_12 = pi/2-abs(self.theta_10)
		else:
			new_theta_12 = pi/2+self.theta_10

		print('New Theta 12 = ', new_theta_12*(180/pi))

		theta_12_adj = self.theta_12-new_theta_12
		print('Theta 12 Adjust =', theta_12_adj*(180/pi))
		self.theta_12 = new_theta_12
		goal = servo6_init-(theta_12_adj*(180/pi))

		return goal

	def equations(self, vars, *args):

		x, y = vars

		eq1 = (args[0]-x)**2+(args[1]-y)**2-self.l4**2
		eq2 = (args[2]-x)**2+(args[3]-y)**2-self.l5**2

		return [eq1, eq2]

	def robot(self):
		'''
		Main function that calibrates the robot and checks the motor connections
		'''
		print('Motors have been checked and are ready to go!')
		print('READY TO WALK')

		print('Servo 1 Angle = ', self.servo1.get_physical_angle())
		print('Servo 2 Angle = ', self.servo2.get_physical_angle())
		print('Servo 3 Angle = ', self.servo3.get_physical_angle())
		print('Servo 4 Angle = ', self.servo4.get_physical_angle())
		print('Servo 5 Angle = ', self.servo5.get_physical_angle())
		print('Servo 6 Angle = ', self.servo6.get_physical_angle())

		self.ready = True

		self.mqtt_client.publish(self.client_name + "/Update", 'READY TO WALK')

	def walk(self):
		pass

	def on_connect(self, client, userdata, flags, rc):
		'''
        Callback function for when the script is connected to the server
        Prints connected to verify
		'''
		print("CONNECTED TO SERVER")

	def on_message(self, client, userdata, msg):
		'''
		Prints message for any message not received through Rigi/ topics
		'''
		message = msg.payload.decode(encoding='UTF-8')
		print(message)

	def on_message_calibrate(self, client, userdata, msg):
		'''
		Callback function for Rigi/calibrate topics
		'''
		message = msg.payload.decode(encoding='UTF-8')
		print(message)

		self.calibrate()

		if not self.calibrated:
			client.publish(self.client_name + "/Update", 'Finished Initial Calibration')
			self.calibrated = True
		else:
			client.publish(self.client_name + "/Update", 'Finished Final Calibration')
			self.final_calibration = True
			self.robot()

	def on_message_left(self, client, userdata, msg):
		'''
        Callback function for Rigi/location topics
        Gets the temperature based on the location that is specified
		'''
		message = msg.payload.decode(encoding='UTF-8')
		print(message)
		#split on the comma because the message in the form of "latitude, longitude"

		self.check_left_motors()
		client.publish(self.client_name + "/Update", 'Left Motors Checked')
		self.right_motors_checked = True

	def on_message_right(self, client, userdata, msg):
		'''
        Callback function for Rigi/location topics
        Gets the temperature based on the location that is specified
		'''
		message = msg.payload.decode(encoding='UTF-8')
		print(message)

		self.check_right_motors()
		client.publish(self.client_name + "/Update", 'Right Motors Checked')
		self.left_motors_checked = True

	def on_message_walk(self, client, userdata, msg):
		'''
        Callback function for Rigi/location topics
        Gets the temperature based on the location that is specified
		'''
		message = msg.payload.decode(encoding='UTF-8')
		print(message)
		#split on the comma because the message in the form of "latitude, longitude"
        
		if self.ready:
			print('Function not implemented yet')
			client.publish(self.client_name + "/Update", 'Function not implemented yet')
		else:
			print('Not ready to walk')
			client.publish(self.client_name + "/Update", 'Not ready to walk')



if __name__ == '__main__':
	RobotController()


