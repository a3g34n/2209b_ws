#!/usr/bin/env python3
#---------------------------------------------------
from pid import PID
import math
import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray, Float32
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
import time


#---------------------------------------------------
def control_kwad(msg, args):
	#Declare global variables as you dont want these to die, reset to zero and then re-initiate when the function is called again.
	global roll, pitch, yaw, err_roll, err_pitch, err_yaw, err_z, x_init, y_init, theta
	
	#Assign the Float64MultiArray object to 'f' as we will have to send data of motor velocities to gazebo in this format
	f = Float64MultiArray()
	
	#Convert the quaternion data to roll, pitch, yaw data
	#The model_states contains the position, orientation, velocities of all objects in gazebo. In the simulation, there are objects like: ground, Contruction_cone, quadcopter (named as 'Kwad') etc. So 'msg.pose[ind]' will access the 'Kwad' object's pose information i.e the quadcopter's pose.
	ind = msg.name.index('Kwad')
	orientationObj = msg.pose[ind].orientation
	orientationList = [orientationObj.x, orientationObj.y, orientationObj.z, orientationObj.w]

	poseObj = msg.pose[ind].position
	x = poseObj.x
	y = poseObj.y
	z = poseObj.z

	(roll, pitch, yaw) = (euler_from_quaternion(orientationList))
	
	#Circular Motion Calculations
	# r = 5
	# if (z < 1):
	# 	theta = 0
	# 	x_init = x
	# 	y_init = y
	# x_set = x_init + (math.cos(theta) * r)
	# y_set = y_init + (math.sin(theta) * r)
	if(z<5):
		x_set = 0
		y_set = 0
	
	else:
		x_set = 0
		y_set = 0

	z_set = 5
	# theta += 0.000001
	# print(x_set)
	#send roll, pitch, yaw data to PID() for attitude-stabilisation, along with 'f', to obtain 'fUpdated'
	#Alternatively, you can add your 'control-file' with other algorithms such as Reinforcement learning, and import the main function here instead of PID().
	(fUpdated, err_roll, err_pitch, err_yaw, err_z, err_x, err_y) = PID(roll, pitch, yaw, f, z, x, y, x_set, y_set, z_set)
	
	#The object args contains the tuple of objects (velPub, err_rollPub, err_pitchPub, err_yawPub. publish the information to namespace.
	args[0].publish(fUpdated)
	args[1].publish(err_roll)
	args[2].publish(err_pitch)
	args[3].publish(err_yaw)
	args[4].publish(err_z)
	args[5].publish(err_x)
	args[6].publish(err_y)
	errorRoll.append(err_roll)
	errorPitch.append(err_pitch)
	errorYaw.append(err_yaw)
	errorX.append(err_x)
	errorY.append(err_y)
	errorZ.append(err_z)
	#print("Roll: ",roll*(180/3.141592653),"Pitch: ", pitch*(180/3.141592653),"Yaw: ", yaw*(180/3.141592653))
	#print(orientationObj)
#----------------------------------------------------
#Initiate the node that will control the gazebo model
errorRoll = []
errorPitch = []
errorYaw = []
errorX = []
errorY = []
errorZ = []

rospy.init_node("Control")
rate = rospy.Rate(50) # ROS Rate at 5Hz
currTime = time.time()
#initiate publishers that publish errors (roll, pitch,yaw - setpoint) so that it can be plotted via rqt_plot /err_<name>  
err_rollPub = rospy.Publisher('err_roll', Float32, queue_size=1)
err_pitchPub = rospy.Publisher('err_pitch', Float32, queue_size=1)
err_yawPub = rospy.Publisher('err_yaw', Float32, queue_size=1)
err_zPub = rospy.Publisher('err_z', Float32, queue_size=1)
err_xPub = rospy.Publisher('err_x', Float32, queue_size=1)
err_yPub = rospy.Publisher('err_y', Float32, queue_size=1)

#initialte publisher velPub that will publish the velocities of individual BLDC motors
velPub = rospy.Publisher('/Kwad/joint_motor_controller/command', Float64MultiArray, queue_size=4)

#Subscribe to /gazebo/model_states to obtain the pose in quaternion form
#Upon receiveing the messages, the objects msg, velPub, err_rollPub, err_pitchPub and err_yawPub are sent to "control_kwad" function.
PoseSub = rospy.Subscriber('/gazebo/model_states',ModelStates,control_kwad,(velPub, err_rollPub, err_pitchPub, err_yawPub, err_zPub, err_xPub, err_yPub))

rate.sleep()
rospy.spin()

plt.figure()
plt.plot(errorRoll)
# naming the x axis
plt.xlabel('Time')
# naming the y axis
plt.ylabel('Error')
  
# giving a title to my graph
plt.title('Roll error')

plt.figure()

plt.plot(errorPitch)
# naming the x axis
plt.xlabel('Time')
# naming the y axis
plt.ylabel('Error')
  
# giving a title to my graph
plt.title('Pitch error')

plt.figure()

plt.plot(errorYaw)
# naming the x axis
plt.xlabel('Time')
# naming the y axis
plt.ylabel('Error')
  
# giving a title to my graph
plt.title('Yaw error')

plt.figure()

plt.plot(errorX)
# naming the x axis
plt.xlabel('Time')
# naming the y axis
plt.ylabel('Error')
  
# giving a title to my graph
plt.title('X error')

plt.figure()

plt.plot(errorY)
# naming the x axis
plt.xlabel('Time')
# naming the y axis
plt.ylabel('Error')
  
# giving a title to my graph
plt.title('Y error')

plt.figure()

plt.plot(errorZ)
# naming the x axis
plt.xlabel('Time')
# naming the y axis
plt.ylabel('Error')
  
# giving a title to my graph
plt.title('Z error')

# function to show the plot
plt.show()
