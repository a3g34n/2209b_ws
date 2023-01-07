#-------------
import time

#--------------------------------------------------------------------------------------------------------------------------------------
def PID(roll, pitch, yaw, f, z, x, y, x_set, y_set):
	#Define the global variables to prevent them from dying and resetting to zero, each time a function call occurs. Some of these variables 		may be redundant.
	global kp_roll, ki_roll, kd_roll, kp_pitch, ki_pitch, kd_pitch, kp_yaw, ki_yaw, kd_yaw,kp_z, ki_z, kd_z, kp_x, ki_x, kd_x, kp_y, ki_y, kd_y ,prevErr_roll, prevErr_pitch, prevErr_yaw, prevErr_z, prevErr_x, prevErr_y, pMem_roll, pMem_yaw, pMem_pitch, pMem_z, pMem_x, pMem_y, iMem_roll, iMem_pitch, iMem_yaw, iMem_z, iMem_x, iMem_y, dMem_roll, dMem_pitch, dMem_yaw, dMem_z, dMem_x, dMem_y, flag, setpoint, setpoint_z, setpoint_x, setpoint_y, sampleTime
	#-----------------------
	#Assign your PID values here. From symmetry, control for roll and pitch is the same.
	kp_roll = 80
	ki_roll = 0.0002
	kd_roll = 89
	kp_pitch = 70
	ki_pitch = 0.0002
	kd_pitch = 89
	kp_yaw = 0.1
	ki_yaw = 0
	kd_yaw = 0

	kp_x = 60
	ki_x = 0.0002
	kd_x = 50
	kp_y = 60
	ki_y = 0.0002
	kd_y = 50
	kp_z = 50
	ki_z = 0.0002
	kd_z = 90

	flag = 0
	#Define other variables here, and calculate the errors.
	sampleTime = 0
	setpoint = 0
	setpoint_x = x_set
	setpoint_y = y_set
	setpoint_z = 5 #EA

	err_pitch = float(pitch)*(180 / 3.141592653) - setpoint 
	err_roll = float(roll)*(180 / 3.141592653) - setpoint
	err_yaw = float(yaw)*(180/3.14159263) - setpoint
	err_x = x - setpoint_x
	err_y = y - setpoint_y
	err_z = z - setpoint_z
	currTime = time.time()
	#-----------------------
	#Reset the following variables during the first run only.
	if flag == 0:
		prevTime = 0
		prevErr_roll = 0
		prevErr_pitch = 0
		prevErr_yaw = 0
		prevErr_z = 0
		prevErr_x = 0
		prevErr_y = 0
		pMem_roll = 0
		pMem_pitch = 0
		pMem_yaw = 0
		pMem_z = 0
		pMem_x = 0
		pMem_y = 0
		iMem_roll = 0
		iMem_pitch = 0
		iMem_yaw = 0
		iMem_z = 0
		iMem_x = 0
		iMem_y = 0
		dMem_roll = 0
		dMem_pitch = 0
		dMem_yaw = 0
		dMem_z = 0
		dMem_x = 0
		dMem_y = 0
		flag += 1
	#------------------------
	#Define dt, dy(t) here for kd calculations.
	dTime = currTime - prevTime
	dErr_pitch = err_pitch - prevErr_pitch
	dErr_roll = err_roll - prevErr_roll
	dErr_yaw = err_yaw - prevErr_yaw
	dErr_z = err_z - prevErr_z
	dErr_x = err_x - prevErr_x
	dErr_y = err_y - prevErr_y

	#-------------------------------------------------------------------------------------------------------------------------------
	#This is the Heart of the PID algorithm. PID behaves more accurately, if it is sampled at regular intervals. You can change the sampleTime to whatever value is suitable for your plant.
	if(dTime >= sampleTime):
		#Kp*e(t)
		pMem_roll = kp_roll * err_roll
		pMem_pitch = kp_pitch * err_pitch
		pMem_yaw = kp_yaw * err_yaw
		pMem_z = kp_z * err_z
		pMem_x = kp_x * err_x
		pMem_y = kp_y * err_y
		
		#integral(e(t))
		iMem_roll += err_pitch * dTime
		iMem_pitch += err_roll * dTime
		iMem_yaw += err_yaw * dTime
		iMem_z += err_z * dTime
		iMem_x += err_x * dTime
		iMem_y += err_y * dTime
		
		if(iMem_roll > 400): iMem_roll = 400
		if(iMem_roll < -400): iMem_roll = -400
		if(iMem_pitch > 400): iMem_pitch = 400
		if(iMem_pitch < -400): iMem_pitch = -400
		if(iMem_yaw > 400): iMem_yaw = 400
		if(iMem_yaw < -400): iMem_yaw = -400
		if(iMem_z > 400): iMem_z = 400
		if(iMem_z < -400): iMem_z = -400
		if(iMem_x > 400): iMem_x = 400
		if(iMem_x < -400): iMem_x = -400
		if(iMem_y > 400): iMem_y = 400
		if(iMem_y < -400): iMem_y = -400

		#derivative(e(t))
		dMem_roll = dErr_roll / dTime
		dMem_pitch = dErr_pitch / dTime
		dMem_yaw = dErr_yaw / dTime
		dMem_z = dErr_z / dTime
		dMem_x = dErr_x / dTime
		dMem_y = dErr_y / dTime
	
	#Store the current variables into previous variables for the next iteration.
	prevTime = currTime
	prevErr_roll = err_roll
	prevErr_pitch = err_pitch
	prevErr_yaw = err_yaw
	prevErr_z = err_z
	prevErr_x = err_x
	prevErr_y = err_y
	
	#output = Kp*e(t) + Ki*integral(e(t)) + Kd*derivative(e(t))
	output_roll = pMem_roll + ki_roll * iMem_roll + kd_roll * dMem_roll
	output_pitch = pMem_pitch + ki_pitch * iMem_pitch + kd_pitch * dMem_pitch
	output_yaw = pMem_yaw + ki_yaw * iMem_yaw + kd_yaw * dMem_yaw
	output_z = pMem_z + ki_z * iMem_z + kd_z * dMem_z
	output_x = pMem_x + ki_x * iMem_x + kd_x * dMem_x
	output_y = pMem_y + ki_y * iMem_y + kd_y * dMem_y

	#-------------------------------------------------------------------------------------------------------------------------------
		#Ignore this.
	#br_motor_vel = 50.5 + output_pitch + output_roll + output_yaw
	#bl_motor_vel = 50.5 - output_pitch + output_roll - output_yaw
	#fl_motor_vel = 50.5 - output_pitch - output_roll + output_yaw
	#fr_motor_vel = 50.5 + output_pitch - output_roll - output_yaw
	
	#-------------------------------------------------------------------------------------------------------------------------------
	#Some Gazebo information for your reference.
	
	#Positive roll is right wing down
	#Positive pitch is front nose down
	#Positive yaw is rotate CCW about z-axis
	
	#Red is x-axis
	#Green is y-axis
	#Blue is z-axis
	
	#-------------------------------------------------------------------------------------------------------------------------------
	#br: Back Right
	#bl: Back Left
	#fl: Front Left
	#fr: Front Right
	#Calculate the ESC pulses (1000us - 2000us PWM signal) for each of the motor.
	
	#br in my code is fr in gazebo's world
	esc_br = 1500 + output_roll + output_pitch - output_yaw - output_z - output_y + output_x
	#bl in my code is br in gazebo's world
	esc_bl = 1500 + output_roll - output_pitch + output_yaw - output_z - output_y - output_x
	#fl in my code is bl in gazebo's world
	esc_fl = 1500 - output_roll - output_pitch - output_yaw - output_z + output_y - output_x
	#fr in my code is fl in gazebo's world
	esc_fr = 1500 - output_roll + output_pitch + output_yaw - output_z + output_y + output_x
	#Limit the ESC pulses to upper limit and lower limit, in case the PID algorithm goes crazy and high af.
	if(esc_br > 2000): esc_br = 2000
	if(esc_bl > 2000): esc_bl = 2000
	if(esc_fr > 2000): esc_fr = 2000
	if(esc_fl > 2000): esc_fl = 2000
	
	if(esc_br < 1100): esc_br = 1100
	if(esc_bl < 1100): esc_bl = 1100
	if(esc_fr < 1100): esc_fr = 1100
	if(esc_fl < 1100): esc_fl = 1100
	
	#Map the esc values to motor values
	br_motor_vel = ((esc_br - 1500)/25) + 50
	bl_motor_vel = ((esc_bl - 1500)/25) + 50
	fr_motor_vel = ((esc_fr - 1500)/25) + 50
	fl_motor_vel = ((esc_fl - 1500)/25) + 50
	#----------------------------------------------------------------------------------------------------------------------------------
	#Ignore this shit here.
	'''
	if(fl_motor_vel > 70): fl_motor_vel = 70
	if(fr_motor_vel > 70): fr_motor_vel = 70
	if(bl_motor_vel > 70): bl_motor_vel = 70
	if(br_motor_vel > 70): br_motor_vel = 70
	
	
	if(err_roll > 0 && err_pitch > 0):
		fl_motor_vel = 51
		fr_motor_vel = 45
		bl_motor_vel = 51
		br_motor_vel = 51
	elif(err_roll > 0 && err_pitch < 0):
		fl_motor_vel = 45
		fr_motor_vel = 51
		bl_motor_vel = 51
		br_motor_vel = 51
	elif(err_roll < 0 && err_pitch > 0):
	'''	
	#---------------------------------------------------------------------------------------------------------------------------------
	#Provide the motor velocities to the object 'f' that will now exit out of this function, and gets published to gazebo, providing velocities to each motor. Note that the sign here is +,-,+,- i.e CW, CCW, CW, CCW in gazebo model. Change view of gazebo model (by scrolling) such that the green line comes to your left, red line goes forward, and blue line goes upward. This is the convention that i refer to as "Gazebo model" incase you get confused.
	f.data = [fr_motor_vel,-fl_motor_vel,bl_motor_vel, -br_motor_vel]
	
	#Return these variables back to the control file.
	return f, err_roll, err_pitch, err_yaw, err_z, err_x, err_y
#--------------------------------------------------------------------------------------------------------------------------------------
