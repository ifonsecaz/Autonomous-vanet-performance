#ifndef GLOBALS_H
#define GLOBALS_H

#include <cv_bridge/cv_bridge.h> 

extern int RATE_HZ; // Declare the global variable
extern int ROI;
extern int RECTANGLE_COLOR;
extern cv_bridge::CvImage img_bridge;

//----Ros publishers----//
//Se definen los publicadores para usarlos dentro de las funciones
ros::Publisher detec_publisher; //Publica la imagen con las detecciones
ros::Publisher detec_publisherk; //Publica la imagen con las predicciones 

#endif