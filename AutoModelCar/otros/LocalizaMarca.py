#! /usr/bin/env python

import rospy 
from sensor_msgs.msg import PointCloud2, PointField
import time
import numpy as np
#import ros_numpy as rnp
#from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from std_msgs.msg import String
import struct
import ctypes



def camaraRGBCallback(msg):
    #depth = rnp.point_cloud2.get_xyz_points(msg)

    global xyz
    global rgb
    global height
    global width
    
    xyz = np.array([[0,0,0]])
    rgb = np.array([[0,0,0]])
    #self.lock.acquire()
    gen = PointCloud2.read_points(msg, skip_nans=True)
    int_data = list(gen)
    height=msg.height
    width=msg.width
    
    #gen = point_cloud2.read_points(data)
    #for p in gen:
        #4 elementos x, y, z, el último corresponde a rgb[3]
        
    for x in int_data:
        test = x[3] 
        # cast float32 to int so that bitwise operations are possible
        s = struct.pack('>f' ,test)
        i = struct.unpack('>l',s)[0]
        # you can get back the float value by the inverse operations
        pack = ctypes.c_uint32(i).value
        r = (pack & 0x00FF0000)>> 16
        g = (pack & 0x0000FF00)>> 8
        b = (pack & 0x000000FF)
        # prints r,g,b values in the 0-255 range
        # x,y,z can be retrieved from the x[0],x[1],x[2]
        xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)
        rgb = np.append(rgb,[[r,g,b]], axis = 0)
        #print(x[0])
        #print(x[3])
        #print(x)
        #print(r)
#Es un solo array [[x1,y1,z1],[x2,y2,z2]]
#[[r1,g1,b1],[r2,g2,b2]]
#(N,3)

def localizaMarca():
    global rgb
    global xyz
    global x
    global y
    global z
    
    #Marca azul
    (N,_)=rgb.shape
    b=rgb[:,3].reshape(height,-1)
    b=b>=40  #threshold
    
    col=np.sum(b[:,:],axis=0)
    row=np.sum(b[:,:],axis=1)
    indexC=np.argwhere(col>0)[0][0]#
    indexR=np.argwhere(row>0)[0][0]
    LindexC=np.argwhere(np.flip(col)>0)[0][0]
    LindexR=np.argwhere(np.flip(row)>0)[0][0]
    centerC=int((LindexC+indexC)/2)
    centerL=int((LindexR+indexR)/2)
    punto=centerC*centerL
    x=xyz[punto,1]
    y=xyz[punto,2]
    z=xyz[punto,3]



if __name__ == '__main__':
    try:
        global x
        global y
        global z
        
        rospy.init_node('ubicaPunto', anonymous = True)

        meta_publisher = rospy.Publisher('/meta', Odometry, queue_size = 10)
        #camaraRGB_subscriber = rospy.Subscriber('/base_camera/cameraRGBTF', PointCloud2, camaraRGBCallback)
        camara_subscriber = rospy.Subscriber('/base_camera/cameraTF', PointCloud2, camaraRGBCallback)
        #Puntos
        while(True):
            (x,y)=localizaMarca()
            odometry_message = Odometry()
            odometry_message.pose.pose.position.x=x
            odometry_message.pose.pose.position.y=z
            odometry_message.pose.pose.position.z=0
            odometry_message.pose.pose.orientation.x=0
            odometry_message.pose.pose.orientation.y=0
            odometry_message.pose.pose.orientation.z=0
            odometry_message.pose.pose.orientation.w=0
            
            time.sleep(0.5)
        
    except rospy.ROSInterruptException:        
        pass
