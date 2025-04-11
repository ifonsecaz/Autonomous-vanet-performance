#! /usr/bin/env python

import rospy
from geometry_msgs.msg import TransformStamped
import time
import tf
import numpy as np

x0=0
y0=0
z0=0
x1=0
y1=0
z1=0
q1=0
q2=0
q3=0
q4=0
q10=0
q20=0
q30=0
q40=0

def laserCallback(msg):
    global x1
    global y1
    global z1
    global q10
    global q20
    global q30
    global q40
    
    x1=msg.transform.translation.x;
	y1=msg.transform.translation.y;
	z1=msg.transform.translation.z;
	q10=-1*msg.transform.rotation.x;
	q20=-1*msg.transform.rotation.y;
	q30=-1*msg.transform.rotation.z;
	q40=-1*msg.transform.rotation.w;
    
def odomCallback(msg):
    global x0
    global y0
    global z0
    global q1
    global q2
    global q3
    global q4
    
    x0=msg.transform.translation.x;
	y0=msg.transform.translation.y;
	z0=msg.transform.translation.z;
	q1=msg.transform.rotation.x;
	q2=msg.transform.rotation.y;
	q3=msg.transform.rotation.z;
	q4=msg.transform.rotation.w;
    
if __name__ == '__main__':
	try:
        global x1
        global y1
        global z1
        global q10
        global q20
        global q30
        global q40
        global x0
        global y0
        global z0
        global q1
        global q2
        global q3
        global q4
    
		rospy.init_node('odomLaserTF', anonymous = True)

		TF_publisher = rospy.Publisher('TFLaserOdom', TransformStamped, queue_size = 10)
		laser_subscriber = rospy.Subscriber('/base_laser/laserTF', TransformStamped, laserCallback)
  		odom_subscriber = rospy.Subscriber('/odomInv', TransformStamped, odomCallback)
        
        T0=[[0,0,0,x0],[0,0,0,y0],[0,0,0,z0],[0,0,0,1]];
        T1=[[0,0,0,-x1],[0,0,0,-y1],[0,0,0,-z1],[0,0,0,1]];
        
        
        # First row of the rotation matrix
        r00 = 2 * (q4 * q4 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q4 * q3)
        r02 = 2 * (q1 * q3 + q4 * q2)
        r03 = 0 
         
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q4 * q3)
        r11 = 2 * (q4 * q4 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q4 * q1)
        r13 = 0 
         
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q4 * q2)
        r21 = 2 * (q2 * q3 + q4 * q1)
        r22 = 2 * (q4 * q4 + q3 * q3) - 1
        r23 = 0
        
        # Third row of the rotation matrix
        r40 = 0
        r41 = 0
        r42 = 0
        r43 = 1
        
        # 4x4 rotation matrix
        rot_matrix = np.array([[r00, r01, r02, r03],
                               [r10, r11, r12, r13],
                               [r20, r21, r22, r23],
                               [r40, r41, r42, r43])
                               

        # First row of the rotation matrix
        r00 = 2 * (q40 * q40 + q10 * q10) - 1
        r01 = 2 * (q10 * q20 - q40 * q30)
        r02 = 2 * (q10 * q30 + q40 * q20)
        r03 = 0 
         
        # Second row of the rotation matrix
        r10 = 2 * (q10 * q20 + q40 * q30)
        r11 = 2 * (q40 * q40 + q20 * q20) - 1
        r12 = 2 * (q20 * q30 - q40 * q10)
        r13 = 0 
         
        # Third row of the rotation matrix
        r20 = 2 * (q10 * q30 - q40 * q20)
        r21 = 2 * (q20 * q30 + q40 * q10)
        r22 = 2 * (q40 * q40 + q30 * q30) - 1
        r23 = 0
        
        # Third row of the rotation matrix
        r40 = 0
        r41 = 0
        r42 = 0
        r43 = 1
        
        # 4x4 rotation matrix
        rot_matrix2 = np.array([[r00, r01, r02, r03],
                               [r10, r11, r12, r13],
                               [r20, r21, r22, r23],
                               [r40, r41, r42, r43])
        
        H_matrix=rot_matrix2*T0
        H_matrix2=rot_matrix*T1
        
        HF=H_matrix2*H_matrix
        q = tf.quaternion_from_matrix(HF)
            
        laserOdom_trans.transform.translation.x = HF[1,4];
        laserOdom_trans.transform.translation.y = vectorTF[2,4];
        laserOdom_trans.transform.translation.z = vectorTF[3,4];
        laserOdom_trans.transform.rotation = q;
        
        transf = tf.TransformBroadcaster()
        
	except rospy.ROSInterruptException:        
		pass
