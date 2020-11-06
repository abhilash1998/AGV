#!/usr/bin/env python
#-------------
import time
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int16MultiArray
#from std_msgs.msg import MultiArrayDimension
#from std_msgs.msg import Float32
import rospy



class Controls:
	
#--------------------------------------------------------------------------------------------------------------------------------------
	def __init__(self) :
		self.output_l=0
		self.output_r=0
		self.old_vel_pos_L=0
		self.old_vel_pos_R=0
		#self.velPub = rospy.Publisher('/VelPub', Int16MultiArray, queue_size=1)
		self.f=Int16MultiArray()
		self.f.data=[0 , 0]
		#self.velPubL.publish(10)
		#self.velPubR = rospy.Publisher('/speedR', Float32, queue_size=1)
	def pid(self,data,args):
		#Define the global variables to prevent them from dying and resetting to zero, each time a function call occurs. Some of these variables 		may be redundant.
		global kp,kd ,ki,  prevErr_r, prevErr_l,pMem_r, pMem_l,iMem_r, iMem_l,dMem_r, dMem_l,  flag, setpoint, sampleTime
		
		#f.layout.dim.append(MultiArrayDimension())
		#f.layout.dim[0].size=2
		#-----------------------
		#Assign your PID values here. From symmetry, control for r and l is the same.
		#print(1)
		current_l=data.data[1]
		current_r=data.data[2]
		newPosition_L=current_l
		newPosition_R=current_r
		# kp = 0.8#100
		# ki = 0.0#2000
		# kd = 0.088#80.5

		kp = 5*0.01
		ki =150*0.01
		kd = 10000*0.01

		setpoint=10
		flag = 0
		#Define other variables here, and calculate the errors.
		sampleTime = 10
		#print(2)
		vel_ticks_L=60.0*((newPosition_L-self.old_vel_pos_L)/288.0)/0.01
		#vel_ticks_R=60.0*((newPosition_L-self.old_vel_pos_L)/288.0)/0.01
		vel_ticks_R=60.0*((newPosition_R-self.old_vel_pos_R)/288.0)/0.01

		


		print("vel_ticks ",vel_ticks_L)
		self.old_vel_pos_L=newPosition_L
		self.old_vel_pos_R=newPosition_R
		#err_r = current_l - setpoint
		err_r = ( setpoint - vel_ticks_R)
		
		
		err_l = (setpoint-vel_ticks_L  )

		currTime = time.time()
		#-----------------------
		#Reset the following variables during the first run only.
		if flag == 0:
			prevTime = 0
			prevErr_r = 0
			prevErr_l = 0
			
			pMem_r = 0
			pMem_l = 0
			
			iMem_r = 0
			iMem_l = 0
			
			dMem_r = 0
			dMem_l = 0
			
			flag += 1
		#------------------------
		#Define dt, dy(t) here for kd calculations.
		dTime = currTime - prevTime
		dErr_l = err_l - prevErr_l
		dErr_r = err_r - prevErr_r
		
		
		#-------------------------------------------------------------------------------------------------------------------------------
		#This is the Heart of the PID algorithm. PID behaves more accurately, if it is sampled at regular intervals. You can change the sampleTime to whatever value is suitable for your plant.
		if(dTime >= sampleTime):
			#Kp*e(t)
			pMem_r = kp * err_r
			pMem_l = kp * err_l
			
			#integral(e(t))
			iMem_r += err_r 
			iMem_l += err_l 
			
			
			if(iMem_r > 30): iMem_r = 30
			if(iMem_r < -30): iMem_r = -30
			if(iMem_l > 30): iMem_l = 30
			if(iMem_l < -30): iMem_l = -30
			
			
			#derivative(e(t))
			dMem_r = dErr_r / dTime
			dMem_l = dErr_l / dTime
			
			print("err_r",err_r)
			#Store the current variables into previous variables for the next iteration.
			prevTime = currTime
			prevErr_r = err_r
			prevErr_l = err_l
																	
			
			#output = Kp*e(t) + Ki*integral(e(t)) + Kd*derivative(e(t))
			self.output_r = setpoint + pMem_r + ki * iMem_r + kd * dMem_r
			self.output_l =setpoint + pMem_l + ki * iMem_l + kd * dMem_l
			#self.output_l= 
			self.output_l=max(min(self.output_l,125),0)
			self.output_r=max(min(self.output_l,125),0)
			print("output_r",self.output_r)
			if(setpoint>0):
				self.f.data=[abs(self.output_l ), abs(self.output_r)]
			else :
				self.f.data=[-abs(self.output_l ), -abs(self.output_r)]

			args.publish(self.f)
			#args[1].publish(self.output_r)
			#Return these variables back to the control file.
			
#--------------------------------------------------------------------------------------------------------------------------------------


rospy.init_node("Control")
control=Controls()
##initialte publisher velPub that will publish the velocities 
velPub = rospy.Publisher('/VelPub', Int16MultiArray, queue_size=1)
#velPubR = rospy.Publisher('/speedR', Float32, queue_size=1)
velPub.publish(control.f)
#PID(velPubL,velPubR)
#Subscribe to /encoder to obtain the encoder values 

rospy.Subscriber('/encoder',Int16MultiArray,control.pid,(velPub))
#PoseSub = rospy.Subscriber('/encoder_left',Float64,control.pid)

rospy.spin()

