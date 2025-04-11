# -*- coding: utf-8 -*-
"""
Created on Wed Oct 26 16:05:00 2022

@author: ifons
"""
import numpy as np
x=0
y=0
xgoal=-8.764
ygoal=46.0
theta=1.57
a=True
while(a):
    kh = 1
    desired_angle_goal = np.arctan2(ygoal-y, xgoal-x) #Asumo xgoal parte de la mitad del carro
	#rads
    if desired_angle_goal < 0:
        desired_angle_goal = desired_angle_goal+2*np.pi
    else:
        desired_angle_goal = desired_angle_goal

    if theta<0:
        theta=2*np.pi+theta

    dtheta = desired_angle_goal - theta

    if (dtheta < -1.57 and dtheta >-3.14):
        dtheta = -1.57
    else:
        if(dtheta >1.57 and dtheta <3.14):
            dtheta= 1.57
        else:
            if dtheta <-3.14:
                dtheta = 6.2831+dtheta
            else:
                if dtheta > 3.14:
                    dtheta = dtheta-6.2831

	#Velocidad
    ka = 5.0
		
    distance = np.abs(np.sqrt(((xgoal-x)**2)+((ygoal-y)**2))) #Detenerse 20 cm antes
    linear_speed = -1*ka * distance
    if linear_speed > 1000:
        linear_speed=1000
    else:
        if linear_speed < -1200:
            linear_speed = -1200
    if linear_speed>-50 and linear_speed<0:
        linear_speed=-75

    if (distance < 1): #Dist menor a 1 cm
	    print ('point (', x, ',',  y, ') reached')
	    linear_speed=0.0
	    break                  
		
		
	#steering_message.Int16=kh*dtheta
    steering_message=int(kh*((dtheta*360.00/6.2831)+90)) 

    print('ster')
    print(dtheta)
    print(steering_message)        

	#velocity_message.Int16 = linear_speed
    velocity_message=int(linear_speed)
    print('vel')
    print(distance)
    print(velocity_message)     
    a=False
