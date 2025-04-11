#include "globals.h"

//----Global variables----//
//Define la taza a la que se publica las detecciones, qué altura considerar
//img guarda la última imagen publicada por la cámara en formato MAT para OpenCV
int RATE_HZ = 10;
int ROI=400;
int RECTANGLE_COLOR=200;

extern cv_bridge::CvImage img_bridge;

//----Ros publishers----//
//Se definen los publicadores para usarlos dentro de las funciones
ros::Publisher detec_publisher; //Publica la imagen con las detecciones
ros::Publisher detec_publisherk; //Publica la imagen con las predicciones 