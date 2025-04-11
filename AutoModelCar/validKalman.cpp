#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/ml.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <time.h>
using namespace cv;
using namespace cv::ml;
using namespace std;

void kalman(KalmanFilter kf, Rect detections, Mat img, int i, bool found, double time, KalmanFilter* nuevo) {
    //KalmanFilter(int  	dynamParams,        int  	measureParams,        int  	controlParams = 0,        int  	type = CV_32F)
    //correct
    //predict
    //Mat prediction = KF.predict();
    //Calculo de la velocidad
    double dt = time;
    kf.transitionMatrix = (Mat_<float>(6, 6) << 1, 0, dt, 0, 0, 0, 0, 1, 0, dt, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1);

    Mat measurement = Mat_<float>::zeros(4, 1);
    Mat state(6, 1, CV_32F); //(6,1)?
    //double dT = 1 / 5;
    //dT=1/fps*algo
    //Mat img2;
    //int k = img.channels();
    //cvtColor(img2,img, COLOR_GRAY2BGR);

    //kf.transitionMatrix.at<float>(2) = dT;
    //kf.transitionMatrix.at<float>(9) = dT;

    state = kf.predict();
    //cout << kf.predict();

    //Creo un rectangulo
    Rect predRect;
    predRect.width = state.at<float>(4);
    predRect.height = state.at<float>(5);
    predRect.x = state.at<float>(0);
    //predRect.x = state.at<float>(0) - predRect.width / 2;
    predRect.y = state.at<float>(1);
    //predRect.y = state.at<float>(1) - predRect.height / 2;

    if (found) {

        measurement.at<float>(0) = detections.x;
        measurement.at<float>(1) = detections.y;
        measurement.at<float>(2) = detections.width;
        measurement.at<float>(3) = detections.height;
        cv::rectangle(img, detections, Scalar(0, 255, 0), 2);

        kf.correct(measurement);
    }
    else {
        kf.statePre.copyTo(kf.statePost);
        kf.errorCovPre.copyTo(kf.errorCovPost);
    }
    *nuevo = kf;
    std::cout << "\nPrediccion x: " << predRect.x << " y " << predRect.y << " w " << predRect.width << " h " << predRect.height;

    cv::rectangle(img, predRect, (255, 255, 255), 2);
    cv::rectangle(img, Rect(predRect.x + predRect.width / 2 - 1, predRect.y + predRect.height / 2 - 1, 2, 2), (255, 255, 255), 2);

    string path = "C:/Users/ifons/source/repos/validKalman/validKalman";
    //String b = path + "/res" + std::to_string(i) + ".jpg";
    String b = path + "/Res" + ".jpg";

    imwrite(b, img);
    waitKey(1);

    //Mismo tipo 
    //KF.correct(measurement);
}

//Metodo preliminar que procesa los primeros dos frames
//Solo entra la primera vez
void preparacionKalman(Rect detections, KalmanFilter* kalman) {
    KalmanFilter KF;
    KF.init(6, 4, 0);

    Mat measurement = Mat_<float>::zeros(4, 1);
    measurement.at<float>(0, 0) = detections.x; //Primer punto
    measurement.at<float>(1, 0) = detections.y;
    measurement.at<float>(2, 0) = detections.width;
    measurement.at<float>(3, 0) = detections.height;


    KF.statePre.setTo(0);
    KF.statePre.at<float>(0, 0) = detections.x;
    KF.statePre.at<float>(1, 0) = detections.y;
    KF.statePre.at<float>(2, 0) = 0;
    KF.statePre.at<float>(3, 0) = 0;
    KF.statePre.at<float>(4, 0) = detections.width;
    KF.statePre.at<float>(5, 0) = detections.height;


    KF.statePost.setTo(0);
    KF.statePost.at<float>(0, 0) = detections.x;
    KF.statePost.at<float>(1, 0) = detections.y;
    KF.statePost.at<float>(2, 0) = 0;
    KF.statePost.at<float>(3, 0) = 0;
    KF.statePost.at<float>(4, 0) = detections.width;
    KF.statePost.at<float>(5, 0) = detections.height;


    KF.transitionMatrix = (Mat_<float>(6, 6) << 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1);
    //setIdentity(KF.measurementMatrix);
    KF.measurementMatrix = cv::Mat::zeros(4, 6, CV_32F);
    KF.measurementMatrix.at<float>(0) = 1.0f;
    KF.measurementMatrix.at<float>(7) = 1.0f;
    KF.measurementMatrix.at<float>(16) = 1.0f;
    KF.measurementMatrix.at<float>(23) = 1.0f;

    setIdentity(KF.processNoiseCov, Scalar::all(.00005)); //adjust this for faster convergence - but higher noise
    //KF.processNoiseCov = (Mat_<float>(6, 6) << 0.00005, 0, 0.00005, 0, 0, 0, 0, 0.00005, 0, 0.00005, 0, 0, 0, 0, 0.00005, 0, 0, 0, 0, 0, 0, 0.00005, 0, 0, 0, 0, 0, 0, 0.00005, 0, 0, 0, 0, 0, 0, 0.00005);

    setIdentity(KF.measurementNoiseCov, Scalar::all(0.001)); //Que tanto se confia en la medicion, que tanto espero varíe, 0.01 ok
    //KF.measurementNoiseCov = (Mat_<float>(6, 6) << 1e-1, 0, 0, 0, 0, 0, 0, 1e-1, 0, 0, 0, 0, 0, 0, 1e-1, 0, 0, 0, 0, 0, 0, 1e-1, 0, 0, 0, 0, 0, 0, 1e-1, 0, 0, 0, 0, 0, 0, 1e-1);

    setIdentity(KF.errorCovPost, Scalar::all(0.1)); //Menor se mueve menos, esperas menos movimiento 0.1 ok
    //KF.errorCovPost = (Mat_<float>(6, 6) << 0.1, 0, 0.1, 0, 0, 0, 0, 0.1, 0, 0.1, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0.1);


    *kalman = KF;
}


int main(int argc, char** argv)
{
    Mat img;
    vector<Rect> detec;
    detec.push_back(Rect(300, 200, 10, 10));
    detec.push_back(Rect(310, 205, 9, 9));
    detec.push_back(Rect(320, 209, 8, 8));
    detec.push_back(Rect(330, 215, 7, 7));
    detec.push_back(Rect(390, 240, 6, 6));
    detec.push_back(Rect(410, 260, 6, 6));
    //detec.push_back(Rect(240, 220, 5, 5));
    //detec.push_back(Rect(239, 230, 5, 5));
    //detec.push_back(Rect(238, 250, 5, 5));
    //detec.push_back(Rect(238, 270, 5, 5));
    //detec.push_back(Rect(238, 300, 5, 5));
    //detec.push_back(Rect(238, 340, 5, 5));
    //detec.push_back(Rect(238, 390, 5, 5));
    vector<double> time;
    time.push_back(2);
    time.push_back(2);
    time.push_back(4);
    time.push_back(6);
    time.push_back(5);
    time.push_back(2);
    time.push_back(2);
    time.push_back(3);
    time.push_back(1);
    time.push_back(2);
    time.push_back(2);
    time.push_back(1.4);
    time.push_back(1.2);
    time.push_back(2);
    time.push_back(8);
    time.push_back(2);
    time.push_back(2);
    time.push_back(6);
    time.push_back(2);
    time.push_back(10);




    KalmanFilter a;
    preparacionKalman(detec[0], &a);
    //Mat measurement = Mat::zeros(4,1, CV_32F);
    //setIdentity(a.processNoiseCov, Scalar::all(1e-5));
    //Mat processNoise(6, 6, CV_32F);
    //setIdentity(a.errorCovPost, Scalar::all(1));
    //setIdentity(a.measurementNoiseCov, Scalar::all(1));
    bool val = true;
    Rect detections;
    KalmanFilter b;
    for (int i = 0; i < 20; i++) {
        //img = Mat(480, 640, CV_64F, 0.0);
        if (i == 0) {
            img = Mat(480, 640, CV_64F, 0.0);

            img = Mat::zeros(Size(640, 480), CV_8UC3);
            //cv::Mat image(320, 240, CV_8UC3, cv::Scalar(0, 0, 0));
        }
        else {
            string path = "C:/Users/ifons/source/repos/validKalman/validKalman/Res.jpg";
            img = imread(path);
        }
        /*
        if (i == 3) {
            printf("entre");
            string path ="C:/Users/ifons/source/repos/validKalman/validKalman/res3.jpg" ;
                img = imread(path);
            //}
        }
        if (i == 4) {
            printf("entre");
            string path = "C:/Users/ifons/source/repos/validKalman/validKalman/res4.jpg";
            img = imread(path);
            //}
        }
        */
        if (i < 5) {
            kalman(a, detec[i + 1], img, i, true, time[i], &b);
            a = b;
            val = true;
            detections = detec[i + 1];
        }
        else {
            kalman(a, detec[0], img, i, false, time[i], &b);
            a = b;
            val = false;
            detections = detec[0];
        }
    }
    std::cout << "\nSig: ";
    //Filtro es a
    
    Mat state(6, 1, CV_32F); //(6,1)?
    Rect predRect;
    
    //KalmanFilter c = a.clone();
    KalmanFilter c;
    c.controlMatrix = a.controlMatrix.clone();
    c.errorCovPost = a.errorCovPost.clone();
    c.errorCovPre = a.errorCovPre.clone();
    c.gain = a.gain.clone();
    c.measurementMatrix = a.measurementMatrix.clone();
    c.measurementNoiseCov = a.measurementNoiseCov.clone();
    c.processNoiseCov = a.processNoiseCov.clone();
    c.statePost = a.statePost.clone();
    c.statePre = a.statePre.clone();
    c.transitionMatrix = a.transitionMatrix.clone();
    c.temp1 = a.temp1.clone();
    c.temp2 = a.temp2.clone();
    c.temp3 = a.temp3.clone();
    c.temp4 = a.temp4.clone();
    c.temp5 = a.temp5.clone();
    
    state = c.predict();
    predRect.width = state.at<float>(4);
    predRect.height = state.at<float>(5);
    predRect.x = state.at<float>(0);
    predRect.y = state.at<float>(1);
    c.statePre.copyTo(c.statePost);
    c.errorCovPre.copyTo(c.errorCovPost);

    std::cout << "\nFiltro Copia1: " << predRect.x << " y " << predRect.y << " w " << predRect.width << " h " << predRect.height;


    state = c.predict();
    predRect.width = state.at<float>(4);
    predRect.height = state.at<float>(5);
    predRect.x = state.at<float>(0);
    predRect.y = state.at<float>(1);
    c.statePre.copyTo(c.statePost);
    c.errorCovPre.copyTo(c.errorCovPost);

    std::cout << "\nFiltro Copia2: " << predRect.x << " y " << predRect.y << " w " << predRect.width << " h " << predRect.height;

    state = c.predict();
    predRect.width = state.at<float>(4);
    predRect.height = state.at<float>(5);
    predRect.x = state.at<float>(0);
    predRect.y = state.at<float>(1);
    c.statePre.copyTo(c.statePost);
    c.errorCovPre.copyTo(c.errorCovPost);

    std::cout << "\nFiltro Copia3: " << predRect.x << " y " << predRect.y << " w " << predRect.width << " h " << predRect.height;

    state = c.predict();
    predRect.width = state.at<float>(4);
    predRect.height = state.at<float>(5);
    predRect.x = state.at<float>(0);
    predRect.y = state.at<float>(1);
    c.statePre.copyTo(c.statePost);
    c.errorCovPre.copyTo(c.errorCovPost);

    std::cout << "\nFiltro Copia4: " << predRect.x << " y " << predRect.y << " w " << predRect.width << " h " << predRect.height;

    state = c.predict();
    predRect.width = state.at<float>(4);
    predRect.height = state.at<float>(5);
    predRect.x = state.at<float>(0);
    predRect.y = state.at<float>(1);
    c.statePre.copyTo(c.statePost);
    c.errorCovPre.copyTo(c.errorCovPost);

    std::cout << "\nFiltro Copia4aaa: " << predRect.x << " y " << predRect.y << " w " << predRect.width << " h " << predRect.height;

    state = a.predict();
    predRect.width = state.at<float>(4);
    predRect.height = state.at<float>(5);
    predRect.x = state.at<float>(0);
    predRect.y = state.at<float>(1);
    a.statePre.copyTo(c.statePost);
    a.errorCovPre.copyTo(c.errorCovPost);

    std::cout << "\nFiltro O1: " << predRect.x << " y " << predRect.y << " w " << predRect.width << " h " << predRect.height;

    state = a.predict();
    predRect.width = state.at<float>(4);
    predRect.height = state.at<float>(5);
    predRect.x = state.at<float>(0);
    predRect.y = state.at<float>(1);
    a.statePre.copyTo(c.statePost);
    a.errorCovPre.copyTo(c.errorCovPost);

    std::cout << "\nFiltro O2: " << predRect.x << " y " << predRect.y << " w " << predRect.width << " h " << predRect.height;
}