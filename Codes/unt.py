#!/usr/bin/env python
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
import sys
import math


from geometry_msgs.msg import PoseStamped,Pose
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
import time
from datetime import datetime,timedelta

bridge =CvBridge()  # initialise the object

def imgcallback(ros_image):
    global bridge
    #convert ros image to opencv convertible image 
    try:
        cv_image=bridge.imgmsg_to_cv2(ros_image,'bgr8')
    except CvBridgeError as e:
        print(e)
    # Therefore image is converted to opencv image - the following code does image preocessing
    (rows,columns,channels)=cv_image.shape

    cv2.imshow("Image Window",cv_image)
    cv2.waitKey(1000)



rospy.init_node('imgConverter',anonymous=True)
# include topic of the image 
#initialise image callback function also specify the topic you have to receive data from
image_sub=rospy.Subscriber("/camera/rgb/image_raw",Image,imgcallback)

##################
class drone():
	def __init__(self):
		
		self.pose_1=PoseStamped()
		
		self.cmd1=Twist()
		self.drone_position_1 = [0.0,0.0,0.0]
		self.targets=[[5*math.cos(N*math.pi/180),5*math.sin(N*math.pi/180),5.0] for N in range(1,361,5)]
		
		self.indx = 0 # Index variable
		
		self.setpoint_1 = self.targets[self.indx]


		self.sample_time = 0.500
		
		
		self.Kp = [275,275,275]
		self.Ki = [1,1,1]
		self.Kd = [100,100,100]

		self.error_1=[0,0,0]               
		self.error_sum_1=[0,0,0]           
		self.error_rate_1=[0,0,0]
		self.previous_error_1=[0,0,0] 


		self.out_1=[0,0,0]


		self.pub1 = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
		rospy.Subscriber('/ground_truth_to_tf/pose',PoseStamped,self.extract_1)

	def pose_cb(self, msg):
        	self.pose = msg
	def extract_1(self,msg):
		self.pose_1=msg
		self.drone_position_1[0] = self.pose_1.pose.position.x
		self.drone_position_1[1] = self.pose_1.pose.position.y
		self.drone_position_1[2] = self.pose_1.pose.position.z

	def send(self):
		
		self.cmd1=Twist()

		self.cmd1.linear.x=0.1
		self.cmd1.linear.y=0.1
		self.cmd1.linear.z=0.1
		self.cmd1.angular.x=0.0
		self.cmd1.angular.y=0.0
		self.cmd1.angular.z=0.0

		self.pub1.publish(self.cmd1)
		print("sent")

	def pid(self):

		
		self.error_1 = [dp - setp for dp,setp in zip(self.drone_position_1, self.setpoint_1)] 
		n_sum_1 = [a + b for a, b in zip(self.error_1, self.error_sum_1)]
		n_derr_1 = [a - b for a, b in zip(self.error_1, self.previous_error_1)] 

		
		out_vals_1=[sum(p*q for p, q in zip(a, b)) for a, b in zip(zip(self.error_1,n_sum_1, n_derr_1), zip(self.Kp, self.Ki, self.Kd))]
		
		self.out_1[0]=-out_vals_1[0]  
		self.out_1[1]=-out_vals_1[1]  
		self.out_1[2]=-out_vals_1[2]  

		self.cmd1.linear.x=self.out_1[0]/2000
		self.cmd1.linear.y=self.out_1[1]/2000
		self.cmd1.linear.z=self.out_1[2]/2000

		
		self.previous_error_1=self.error_1   
		
		
		time.sleep(e_drone.sample_time)
		
		x = self.drone_position_1[0] > self.setpoint_1[0] - 0.2 and self.drone_position_1[0] < (self.setpoint_1[0]+0.2) 
		y = self.drone_position_1[1] > self.setpoint_1[1] - 0.2 and self.drone_position_1[1] < (self.setpoint_1[1]+0.2) 
		z = self.drone_position_1[2] > self.setpoint_1[2] - 0.5 and self.drone_position_1[2] < (self.setpoint_1[2]+0.5) 
		if(x and y and z):
			if(self.indx >= 359):                       
				self.setpoint_1 = [2.701511529340699, 4.207354924039483, 5.0]
				self.indx = 360
			else:	
				self.indx += 1		
				self.setpoint_1 = self.targets[self.indx]  
				
			print('Reached level ',self.indx)
			print(self.drone_position_1)
		self.pub1.publish(self.cmd1)

	

####################3
if __name__ == '__main__':

	e_drone = drone()
	r = rospy.Rate(100) 
	
	
	while not rospy.is_shutdown():
		e_drone.pid()
		r.sleep()
	cv2.destroyAllWindows()
