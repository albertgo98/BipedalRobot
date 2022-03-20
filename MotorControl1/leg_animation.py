import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
import math
from scipy.optimize import fsolve


def equations(vars, *args):
	
	x, y = vars

	eq1 = (args[0]-x)**2+(args[1]-y)**2-l2**2
	eq2 = (args[2]-x)**2+(args[3]-y)**2-l3**2

	return [eq1, eq2]

#Define Link Lengths
l1 = 2
l2 = 4.977
l3 = 2
l4 = 4
l5 = 1.5
l6 = 6

#Initial Motorized Joint Angles
theta_1 = -(math.pi/180)*110
theta_4 = -(math.pi/180)*80

#Fixed Joint A Position
A_x = 0
A_y = 0

#Fixed Joint E Position
E_x = 1.5
E_y = 0

#Initial Joint B Position
B_x = l1*math.cos(theta_1)
B_y = l1*math.sin(theta_1)

#Initial Joint D Position
D_x = l5+l4*math.cos(theta_4)
D_y = l4*math.sin(theta_4)

#Initial Joint C Position
C_x = 2.194879908034972 
C_y = -5.93923099142825

theta_3 = -math.acos((C_x-D_x)/math.sqrt(((C_x-D_x)**2+(C_y-D_y)**2)))
print(theta_3)

#Initial Joint F Position
F_x = C_x+l6*math.cos(theta_3)
F_y = C_y+l6*math.sin(theta_3)

def calc_distance(p1, p2):
	'''
	p1: coordinates of the first point; it is a tuple
	p2: coordinates of the second point; it is a tuple

	returns the distance
	'''
	distance = math.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)

	return distance

# Create the figure and the line that we will manipulate
fig, ax = plt.subplots()

link1, = plt.plot([A_x, B_x], [A_y, B_y], color="black")
link2, = plt.plot([C_x, B_x], [C_y, B_y], color="black")
link3, = plt.plot([C_x, D_x], [C_y, D_y], color="black")
link4, = plt.plot([E_x, D_x], [E_y, D_y], color="black")
link6, = plt.plot([C_x, F_x], [C_y, F_y], color="black")

jointA, = plt.plot(A_x, A_y, 'o', markersize=3, color="red")
jointB, = plt.plot(B_x, B_y, 'o', markersize=3, color="red")
jointC, = plt.plot(C_x, C_y, 'o', markersize=3, color="red")
jointD, = plt.plot(D_x, D_y, 'o', markersize=3, color="red")
jointE, = plt.plot(E_x, E_y, 'o', markersize=3, color="red")
jointF, = plt.plot(F_x, F_y, 'o', markersize=3, color="red")

ax.set_xlim(-3, 6)
ax.set_ylim(-15, 1)

# adjust the main plot to make room for the sliders
plt.subplots_adjust(left=0.25, bottom=0.25)

# Make a horizontal slider to control the frequency.
ax_motor1 = plt.axes([0.25, 0.1, 0.65, 0.03])
motor1 = Slider(
    ax=ax_motor1,
    label='Motor 1',
    valmin=-math.pi,
    valmax=math.pi/4,
    valinit=theta_1,
)

# Make a vertically oriented slider to control the amplitude
ax_motor2 = plt.axes([0.1, 0.25, 0.0225, 0.63])
motor2 = Slider(
    ax=ax_motor2,
    label="Motor 2",
    valmin=-7*math.pi/6,
    valmax=0,
    valinit=theta_4,
    orientation="vertical"
)


def update(val):

	global C_x, C_y

	theta_1 = motor1.val
	theta_4 = motor2.val

	B_x = l1*math.cos(theta_1)
	B_y = l1*math.sin(theta_1)

	#Joint D
	D_x = l5+l4*math.cos(theta_4)
	D_y = l4*math.sin(theta_4)

	C_x, C_y = fsolve(equations, (C_x, C_y), args=(B_x, B_y, D_x, D_y))
	# print(C_x, C_y)

	theta_3 = -math.acos((C_x-D_x)/math.sqrt(((C_x-D_x)**2+(C_y-D_y)**2)))

	F_x = C_x+l6*math.cos(theta_3)
	F_y = C_y+l6*math.sin(theta_3)

	print('Link 3 = ', calc_distance((D_x, D_y), (C_x, C_y)))
	print('Link 2 = ', calc_distance((B_x, B_y), (C_x, C_y)))
	print('Link 1 = ', calc_distance((B_x, B_y), (A_x, A_y)))


	link1.set_xdata([A_x, B_x])
	link1.set_ydata([A_y,B_y])

	link2.set_xdata([C_x, B_x])
	link2.set_ydata([C_y, B_y])

	link3.set_xdata([C_x, D_x])
	link3.set_ydata([C_y, D_y])

	link4.set_xdata([E_x, D_x])
	link4.set_ydata([E_y, D_y])

	link6.set_xdata([C_x, F_x])
	link6.set_ydata([C_y, F_y])

	jointB.set_xdata(B_x)
	jointB.set_ydata(B_y)

	jointD.set_xdata(D_x)
	jointD.set_ydata(D_y)

	jointE.set_xdata(E_x)
	jointE.set_ydata(E_y)

	jointC.set_xdata(C_x)
	jointC.set_ydata(C_y)

	jointF.set_xdata(F_x)
	jointF.set_ydata(F_y)

	fig.canvas.draw_idle()

# register the update function with each slider
motor1.on_changed(update)
motor2.on_changed(update)

# Create a `matplotlib.widgets.Button` to reset the sliders to initial values.
resetax = plt.axes([0.8, 0.025, 0.1, 0.04])
button = Button(resetax, 'Reset', hovercolor='0.975')

def reset(event):
	global C_x, C_y
	motor1.reset()
	motor2.reset()

	#Initial Motorized Joint Angles
	theta_1 = -(math.pi/180)*110
	theta_4 = -(math.pi/180)*80

	#Fixed Joint A Position
	A_x = 0
	A_y = 0

	#Fixed Joint E Position
	E_x = 1.5
	E_y = 0

	#Initial Joint B Position
	B_x = l1*math.cos(theta_1)
	B_y = l1*math.sin(theta_1)

	#Initial Joint D Position
	D_x = l5+l4*math.cos(theta_4)
	D_y = l4*math.sin(theta_4)

	#Initial Joint C Position
	C_x = 2.194879908034972 
	C_y = -5.93923099142825

	theta_3 = -math.acos((C_x-D_x)/math.sqrt(((C_x-D_x)**2+(C_y-D_y)**2)))

	#Initial Joint F Position
	F_x = C_x+l6*math.cos(theta_3)
	F_y = C_y+l6*math.sin(theta_3)


button.on_clicked(reset)


plt.show()



