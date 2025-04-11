#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time
import numpy as np
#from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from std_msgs.msg import String
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>

x = 0
y = 0
z = 0
theta = 0

def poseCallback(msg):
	global x
	global y
	global z
	global theta    

	x=msg.pose.pose.position.x
	y=msg.pose.pose.position.y
	xq=msg.pose.pose.orientation.x
	yq=msg.pose.pose.orientation.y
	zq=msg.pose.pose.orientation.z
	wq=msg.pose.pose.orientation.w
	theta=np.arctan2(2*(wq*zq+xq*yq),(1-2*(np.power(yq,2)+np.power(zq,2)))) 

	#Conversion del cuaternion a angulo de euler yaw
	#pitch=np.arcsin(2*(x*z-y*w))
	#roll=np.arctan2(2*(z*y+x*w),(1-2*(np.power(z,2)+np.power(w,2))))
	#yaw=np.arctan2(2*(w*z+x*y),w*w+x*x-y*y-z*z); si el cuaternion no es unitario 
	#No gimbal problems
    
def movimiento (xgoal, ygoal):
    
	global x
	global y
	global theta

	velocity_message = Int16
	cmd_vel_topic = '/manual_control/speed'
	steering_message = Int16
	cmd_ster_topic = '/manual_control/steering'

	while(True):
		kh = 1
		desired_angle_goal = np.arctan2(ygoal-y, xgoal-x)
		#rads
		if desired_angle_goal < 0:
			desired_angle_goal = desired_angle_goal + 2*np.pi
		else:
			desired_angle_goal = desired_angle_goal
            
		if theta<0:
			theta=2*np.pi+theta
            
		dtheta = desired_angle_goal - theta

		if (dtheta < -1.57 and dtheta >-3.14):
            		dtheta = -1.57
        	else:
            		if (dtheta >1.57 and dtheta <3.14):
                		dtheta = 1.57
			else:
                		if dtheta <-3.14:
		                    dtheta = 6.2831+dtheta
                		else:
		                    if dtheta > 3.14:
                		        dtheta = dtheta-6.2831
                        
		#Velocidad
	        ka = 50.0
		
		distance = np.abs(np.sqrt(((xgoal-x)**2)+((ygoal-y)**2)))
		linear_speed = -1*ka * distance

		if linear_speed > 200:
			linear_speed=200
		else:
			if linear_speed < -200:
				linear_speed = -200

		if (distance < 0.1):
		    print ('point (', x, ',',  y, ') reached')
		    linear_speed=0.0
		    time.sleep(1)
		    break                  
		
		
        	#steering_message.Int16=kh*dtheta
	        steering_message=int(kh*((dtheta*360.00/6.2831)+90))
        
	        steering_publisher.publish(steering_message)
		print('ster')
		print(dtheta)
		print(steering_message)        

		#velocity_message.Int16 = linear_speed
		velocity_message=int(linear_speed)
		print('vel')
		print(distance)
		print(velocity_message)        

		velocity_publisher.publish(velocity_message)

        

if __name__ == '__main__':
	try:
		rospy.init_node('controlCoord', anonymous = True)

		cmd_vel_topic = '/manual_control/speed'
		velocity_publisher = rospy.Publisher(cmd_vel_topic, Int16, queue_size = 10)
        	cmd_ster_topic = '/manual_control/steering'
		steering_publisher = rospy.Publisher(cmd_ster_topic, Int16, queue_size = 10)
        	#lights_topic = '/manual_control/lights'
		#lights_publisher = rospy.Publisher(lights_topic, String, queue_size = 10)

		position_topic = "/odom"
		pose_subscriber = rospy.Subscriber(position_topic, Odometry, poseCallback)

		#Puntos
		x0=1.5
		y0=2.5
        
	        movimiento(x0,y0)
		print('N')
        	time.sleep(3)
	        movimiento(4,3)
        	time.sleep(3)
	        movimiento(-1,2)
	        time.sleep(3)
        
		
	except rospy.ROSInterruptException:        
		pass

