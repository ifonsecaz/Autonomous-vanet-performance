# -*- coding: utf-8 -*-
"""
Created on Fri Oct 21 08:20:48 2022

@author: ifons
"""
import numpy as np
from PIL import Image


with open("D:/Users/User/Documents/CarroAutonomo/fileR.txt", "r") as filestream:
    for line in filestream:
        currentline = line.split(",")
    datos=np.asarray(currentline)
    print(datos.shape)
    mat = np.reshape(datos,(480,-1))

# Creates PIL image
    img = Image.fromarray(np.uint8(mat) , 'L')
    img.show()
    #96 son puntos no visibles
    #parte=mat[300:400,100:200]
    #img2 = Image.fromarray(np.uint8(parte) , 'L')
    #img2.show()
    
    #parte2=mat[300:320,120:140]
    #img3 = Image.fromarray(np.uint8(parte2) , 'L')
    #img3.show()
    #Valores hacia abajo 30-50 son correctos
    #parte3=mat[300:460,200:360]
    #img3 = Image.fromarray(np.uint8(parte3) , 'L')
    #img3.show()
    
    b1=datos.reshape(480,-1).astype(np.uint8)



    (height,width)=b1.shape 
    #print(b.tolist())
    aux=b1<=5 #threshold
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
    suma=0
    jrow=0
    while suma<mitad:
        suma=suma+row[jrow]
        jrow=jrow+1
    
    if jrow==height:
        jrow=0
    if icol==width:
        icol=0
    
    b1[jrow,icol]=255
    img = Image.fromarray(np.uint8(b1) , 'L')
    img.show()
    
    punto=icol+jrow*width
    datos[punto]=255
    b2=datos.reshape(480,-1)
    img = Image.fromarray(np.uint8(b2) , 'L')
    img.show()
    
    print(punto)
    
    #CENTRO EN 0.08764