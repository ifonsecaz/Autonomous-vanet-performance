#ifndef TRACKINGFUNCTIONS_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define TRACKINGFUNCTIONS_H

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

/**
+  * @brief Performs Kalman filter-based tracking.
+  * 
+  * @param kf KalmanFilter object for state estimation.
+  * @param detections Detected object bounding box.
+  * @param img Input image for visualization.
+  * @param i Index of the current detection frame.
+  * @param found Boolean indicating if the object was found in the current frame.
+  * @param lastP Timestamp of the last processed frame.
+  * @param Pactual Timestamp of the current frame.
+  * @param act Pointer to the new KalmanFilter for updating state.
+  */
void kalman(
    KalmanFilter kf, 
    Rect* detections, 
    Mat* img,
    int i,
    bool found, 
    std::chrono::steady_clock::time_point* lastP, 
    std::chrono::steady_clock::time_point* Pactual,
    KalmanFilter* act
);

/**
+  * @brief Initializes a Kalman filter.
+  * 
+  * @param kf Pointer to the KalmanFilter object to be initialized.
+  * @param detections Detected object bounding box, initial state.
+  * @param i Index of the current detection frame.
+  * @param cont Counter for tracking initialization steps.
+  */
void preparacionKalman(
    Rect* detections, 
    KalmanFilter* kalman
);

void savePredictionImage(Mat* img, int cont);

void publishPredictedImage(Mat* imgDet);

#endif // TRACKINGFUNCTIONS_H