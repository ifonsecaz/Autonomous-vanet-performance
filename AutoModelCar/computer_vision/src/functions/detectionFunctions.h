#ifndef DETECTIONFUNCTIONS_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define DETECTIONFUNCTIONS_H

#define _USE_MATH_DEFINES
#include <cmath>
#include <unistd.h>
#include <boost/foreach.hpp>
#include <stdint.h>
#include <vector>
#include <iostream>
#include <time.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h> 
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/ml/ml.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/opencv.hpp"

//mensajes
#include "computer_vision/detec.h"
#include "computer_vision/detecTiempos.h"
#include "computer_vision/detecArray.h"

#include "globals.h"

void detecVentana(
    Ptr<SVM>* svm, 
    Ptr<SVM>* svm2,
    HOGDescriptor* hog,
    CascadeClassifier* carC, 
    Mat* img, 
    int cont, 
    Mat* imgF, 
    int* numberDetec, 
    vector<Rect>* detections
);

float predictWithSVM(
    Ptr<SVM>* svm, 
    Ptr<SVM>* svm2,
    HOGDescriptor* hog, 
    Mat* imgRec
);

void detecRegion(
    CascadeClassifier* carC, 
    Mat* img, 
    int cont, 
    Rect predRect,
    Mat* imgF, 
    bool* encontro, 
    Rect* detections
);

void saveDetectionImage(Mat* img, int cont);

void publishDetectionImage(Mat* imgDet);

#endif // DETECTIONFUNCTIONS_H