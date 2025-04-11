#! /usr/bin/env python

import rospy 
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import time
import numpy as np
#import ros_numpy as rnp
#from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import UInt16MultiArray
import struct
import ctypes

x=0
z=0
punto=0


#Se suscribe y guarda los mensajes de la camaraCallback
#No importa que tenga otra frecuencia de publicacion, yo defino el mio
def camaraRGBCallback(msg):
    global msg2
    gen = pc2.read_points(msg, skip_nans=True)
    int_data = list(gen)
    aux=np.asarray(int_data)
    #xA=aux[:,0]
    #yA=aux[:,1]
    #zA=aux[:,2]  
    #rgb=aux[:,3]
    #rgb_message = Float64MultiArray()
    #rgb_message.data=rgb
    #rgb_publisher.publish(rgb_message)
    msg2=msg

#Se extraen los puntos
#Se manda el color
def camaraCallback(msg):
    #depth = rnp.point_cloud2.get_xyz_points(msg)
    global xA
    global yA
    global zA
    global rgb
    global r
    global g
    global b
    global height
    global width
    
    height=msg.height
    gen = pc2.read_points(msg, skip_nans=True)
    int_data = list(gen)
    aux=np.asarray(int_data)
    rgb=aux[:,3]
    rgb_message = Float64MultiArray()
    rgb_message.data=rgb
    rgb_publisher.publish(rgb_message)
    xA=aux[:,0]
    yA=aux[:,1]
    zA=aux[:,2]    

#Se recive el color
def colorCallback(msg):
    global r
    rdata=list(msg.data)
    #print(rdata)
    r=np.asarray(rdata).astype(np.uint8)
    localizaMarca()

#Localizando centro de gravedad
def localizaMarca():
    global r
    global g
    global b
    global xyz
    global x
    global y
    global z
    global height
    global punto
    
    #Marca azul
    #(N,_)=rgb.shape
    b1=r.reshape(height,-1)
    (height,width)=b1.shape 
    #print(b.tolist())
    aux=b1<=5  #threshold
    #print(np.max(b1))    
    col=np.sum(aux[:,:],axis=0)
    row=np.sum(aux[:,:],axis=1)
    total=np.sum(col)
    #print(col.shape)
    mitad=total/2
    suma=0
    icol=0
    while suma<mitad:
        suma=suma+col[icol]
        icol=icol+1
    jrow=0
    suma=0
    while suma<mitad:
        suma=suma+row[jrow]
        jrow=jrow+1
    #Mejoras: excluir valores alejados
    #tamano minimo de objeto/pixeles
    if jrow==height:
        jrow=0
    if icol==width:
        icol=0
    punto=icol+jrow*width
    x=xA[punto]
    y=yA[punto]
    z=zA[punto]
    print(x)
    print(y)
    print(z)
    if(punto!=0):
	    print('NuevaMarca')
        print(100*x-8.764)
	    print('Punto')
        print(punto)
        odometry_message = Odometry()
        odometry_message.pose.pose.position.x=100*x-8.764
        odometry_message.pose.pose.position.y=100*z-15 #Distancia a la que pararse
        odometry_message.pose.pose.position.z=0
        odometry_message.pose.pose.orientation.x=0
        odometry_message.pose.pose.orientation.y=0
        odometry_message.pose.pose.orientation.z=0
        odometry_message.pose.pose.orientation.w=0
        meta_publisher.publish(odometry_message)



if __name__ == '__main__':
    try:
        global x
        global y
        global z
        global msg2
        global punto
        
        rospy.init_node('ubicaPunto', anonymous = True)

        meta_publisher = rospy.Publisher('/meta', Odometry, queue_size = 10)
        rgb_publisher = rospy.Publisher('/argb', Float64MultiArray, queue_size = 10)
        #camaraRGB_subscriber = rospy.Subscriber('/base_camera/cameraRGBTF', PointCloud2, camaraRGBCallback)
        camara_subscriber = rospy.Subscriber('/depth/points', PointCloud2, camaraRGBCallback)
        color_subscriber = rospy.Subscriber('/red', UInt16MultiArray, colorCallback)
        #Puntos
        time.sleep(2)
	    while(True):
            #(x,y)=localizaMarca()
            camaraCallback(msg2)
            
            
            time.sleep(0.2) #5 mensajes por segundo
        
    except rospy.ROSInterruptException:        
        pass
