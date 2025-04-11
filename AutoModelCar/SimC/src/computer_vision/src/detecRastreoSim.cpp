#include <math.h>
#include <cmath>
#include <unistd.h>
#include <boost/foreach.hpp>
#include <stdint.h>
#include <vector>
#include <iostream>
#include <time.h>
#define _USE_MATH_DEFINES

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

//funciones
#include "functions/globals.h"
#include "functions/detectionFunctions.h"
#include "functions/trackingFunctions.h"

using namespace cv;
using namespace cv::ml;
using namespace std;

cv::Mat img= Mat(ROI, 640, CV_64F, 0.0);

//---Hog descriptor configuration---//
//Se prepara el descriptor HOG con los mismos parámetros con los que se entronaron las SVM
HOGDescriptor hog;
hog.winSize = Size(64, 64);
hog.blockSize = Size(16, 16); //
hog.cellSize = Size(4, 4);
hog.blockStride = Size(4, 4);
hog.nbins = 9;
hog.derivAperture = 1;
hog.winSigma = 4;
//hog.histogramNormType = 0;
hog.L2HysThreshold = 2.0000000000000001e-01;
hog.gammaCorrection = 1;
hog.nlevels = 64;
hog.signedGradient = 0;

//Classifiers
Ptr<SVM> svm;
Ptr<SVM> svm2;
CascadeClassifier carC;

//publicador
ros::Publisher detecciones;

//Recibe la imagen de la cámara y la convierta a tipo MAT con cv_bridge, ya recorta la altura y se pasa a escala de grises
void camaraRGBCallback(const sensor_msgs::Image& msg)
{
    Mat img0;
	cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    
    img0 = cv_ptr->image.clone();
	img0(Rect(0, 0, 640, ROI)).copyTo(img);
    std::cout << img.channels();
    //std::cout<<"\n Imagen cargada";
}

/**
 * @brief Performs the detection step for object by detecting the full window or searching predicted regions.
 * 
 * This function is responsible for periodically performing a full detection step
 * or updating the detections based on predictions from Kalman filters. It processes
 * the input image, detects objects, and updates the list of detections.
 * 
 * @param cont Pointer to the frame counter. Used to determine when to perform a full detection.
 * @param numberO Number of objects currently being tracked.
 * @param objetos Pointer to a vector of rectangles representing the regions of interest (ROIs) for tracked objects.
 * @param imgF Pointer to the processed image frame where detections are drawn.
 * @param filtrosK Pointer to a vector of Kalman filters used for predicting object positions.
 * @param detections Pointer to a vector of rectangles where new detections will be stored.
 * @param numberD Pointer to the number of detections found in the current frame.
 * @param i Current frame index.
 * 
 * @details
 * - If the frame counter (`cont`) reaches 5, a full detection is performed using the `detecVentana` function.
 * - Otherwise, the function predicts the positions of tracked objects using Kalman filters and attempts to detect objects
 *   in the predicted regions. If detection fails, it falls back to the previous positions of the objects.
 */
void detectionStep(int* cont, int numberO, vector<Rect>* objetos, Mat* imgF, vector<KalmanFilter>* filtrosK, vector<Rect>* detections, int* numberD, int i){
    if (*cont >= 5) { //cada 5 frames deteccion completa
        //detecVentana(svm, carC, img, i, &imgF, &numberD, &detections); //cargar svm7
        detecVentana(&svm, &svm2,&carC, &img, i, imgF, numberD, detections); //cargar svm7
        *cont = 1; // Reset counter to ensure periodic detection
    }
    else {
        *numberD = 0;
        for (size_t j = 0; j < numberO; j++) {
            //tamano region,x(esquina),y,width,height
            //detections regresa la nueva pos
            //bool si se detecto
            //SOlo busca en la region, pero dibuja el rectangulo en la imagen completa
            Rect detec;
            bool encontro=false;
            
            Mat state(6, 1, CV_32F);

            state = (*filtrosK[j]).predict();

            //Creo un rectangulo
            Rect predRect;
            predRect.width = state.at<float>(4);
            predRect.height = state.at<float>(5);
            predRect.x = state.at<float>(0);
            predRect.y = state.at<float>(1);

            //if(predRect.x < 640 && predRect.x > 0 && predRect.y < ROI && predRect.y > 0){
            detecRegion(&carC, &img, i, predRect, imgF, &encontro, &detec);                    
            //}
            if (encontro) {
                detections->push_back(detec);
                (*numberD)++;
            }
            else {
                //Busca en su posicion anterior
                detecRegion(&carC, &img, i, (*objetos)[j], imgF, &encontro, &detec);
                if (encontro) {
                    detections->push_back(detec);
                    (*numberD)++;
                }
                else{
                    *imgF = img;
                }
            }
            
        }
        (*cont)++;
    }
}

/**
 * @brief Tracks detected objects across frames using Kalman filters.
 * 
 * This function updates the positions of detected objects, associates new detections with new filters,
 * and removes objects that have not been detected for a specified number of frames.
 * 
 * @param numberD The number of detections in the current frame.
 * @param objetos A vector of rectangles representing tracked objects.
 * @param detections A vector of rectangles representing new detections in the current frame.
 * @param ultimoFrame A vector storing the last frame index where each object was detected.
 * @param filtrosK A vector of Kalman filters associated with each tracked object.
 * @param lastPrediction The timestamp of the last prediction step, (previous frame).
 * @param PrediccionActual The current timestamp for prediction.
 * @param numberO A pointer to the total number of tracked objects.
 * @param i The current frame index.
 * @param imgF A pointer to the image where tracking results are drawn.
 */
void trackingStep(int numberD, vector<Rect>* objetos, vector<Rect>* detections, vector<int>* ultimoFrame, vector<KalmanFilter>* filtrosK, std::chrono::steady_clock::time_point* lastPrediction, std::chrono::steady_clock::time_point* PrediccionActual, int* numberO, int i, Mat* imgF) {
    for (size_t j = 0; j < numberD; j++) {
        //Varianza, +-20 pixeles es el mismo objeto
        int k = 0;
        bool resp = false;
        while (k < objetos->size() && !resp) {
            //localizo el centro
            //Identificar si es el mismo vehiculo entre dos frames
            if (abs(((*detections)[j].x+((*detections)[j].width/2))-((*objetos)[k].x+((*objetos)[k].width/2)))<=50 && abs(((*detections)[j].y+((*detections)[j].height/2)) - ((*objetos)[k].y+((*objetos)[k].height/2))) <= 50) {
                resp = true; //Ya esta
            }
            k++;
        }
        if (!resp) {
            objetos->push_back((*detections)[j]); //nuevo
            ultimoFrame->push_back(i);
            KalmanFilter a;
            *lastPrediction = std::chrono::steady_clock::now();
            preparacionKalman((*detections)[j], &a);
            filtrosK->push_back(a);
        }
        else
        {
            (*objetos)[k - 1] = (*detections)[j]; //Predice
            (*ultimoFrame)[k - 1] = i;
        }
    }
    cout << "\nTotal objetos: " << objetos->size();
    for (size_t m = 0; m < objetos->size(); m++) {
        cout << "\n Posicion x: " << (*objetos)[m].x << " y: " << (*objetos)[m].y << " ultimo frame: " << (*ultimoFrame)[m];
    }
    int m = 0;
    KalmanFilter a;

    *PrediccionActual = std::chrono::steady_clock::now();

    while (m < *numberO) {
        if (i == (*ultimoFrame)[m]) {
            //La i se quita, solo para guardar las imagenes
            //Considerar ultimo frame para ya no calcularlo ej i-5
            kalman((*filtrosK)[m], (*objetos)[m], imgF,i,true,PrediccionActual,lastPrediction,&a);
            (*filtrosK)[m] = a;
        }
        else {
            if (i - (*ultimoFrame)[m] < 15) {
                kalman((*filtrosK)[m], (*objetos)[m], imgF, i, false,PrediccionActual,lastPrediction,&a);
                (*filtrosK)[m] = a;
            }
            else
            {
                filtrosK->erase(filtrosK->begin()+m);
                objetos->erase(objetos->begin() + m);
                ultimoFrame->erase(ultimoFrame->begin() + m);
                m--;
                (*numberO)--;
            }
        }
        m++;
    }
}

/**
 * @brief Publishes a message containing detection information and predicted positions.
 * 
 * This function takes the number of detections and a vector of Kalman filters, processes
 * the current state of each Kalman filter, predicts future states, and publishes the 
 * detection information as a message.
 * 
 * @param numberO The number of detections to process.
 * @param filtrosK A pointer to a vector of KalmanFilter objects, each representing 
 *                 the state and parameters of a Kalman filter.
 * 
 * The function performs the following steps:
 * 1. Initializes a `computer_vision::detecArray` object to store detection data.
 * 2. Iterates through the number of detections (`numberO`):
 *    - Creates a deep copy of the current Kalman filter so it doesn't modify the original filters.
 *    - Predicts the current state of the object using the Kalman filter.
 *    - Stores the predicted position (x, y) and size (width, height) in a `computer_vision::detec` object.
 *    - Predicts the next 10 positions of the object and stores them in a `computer_vision::detecTiempos` object.
 *    - Appends the detection data to the `detecArray`.
 * 3. Publishes the `detecArray` message using the `detecciones` publisher.
 */
void publishMessage(int numberO, vector<KalmanFilter>* filtrosK){
    computer_vision::detecArray det; //
	det.numDetec=numberO;
	
	int m=0;
	while(m<numberO){
        //deep copy of kalman filter
        computer_vision::detecTiempos detT;
		KalmanFilter c;
    	c.controlMatrix = (*filtrosK)[m].controlMatrix.clone();
		c.errorCovPost = (*filtrosK)[m].errorCovPost.clone();
		c.errorCovPre = (*filtrosK)[m].errorCovPre.clone();
		c.gain = (*filtrosK)[m].gain.clone();
		c.measurementMatrix = (*filtrosK)[m].measurementMatrix.clone();
		c.measurementNoiseCov = (*filtrosK)[m].measurementNoiseCov.clone();
    	c.processNoiseCov = (*filtrosK)[m].processNoiseCov.clone();
		c.statePost = (*filtrosK)[m].statePost.clone();
		c.statePre = (*filtrosK)[m].statePre.clone();
		c.transitionMatrix = (*filtrosK)[m].transitionMatrix.clone();
		c.temp1 = (*filtrosK)[m].temp1.clone();
		c.temp2 = (*filtrosK)[m].temp2.clone();
		c.temp3 = (*filtrosK)[m].temp3.clone();
		c.temp4 = (*filtrosK)[m].temp4.clone();
		c.temp5 = (*filtrosK)[m].temp5.clone();

		computer_vision::detec detecInd; 
		//Rect detecInd;
		Mat state(6, 1, CV_32F);
		state = c.predict();
    	detecInd.width = state.at<float>(4);
    	detecInd.height = state.at<float>(5);
    	detecInd.x = state.at<float>(0);
    	detecInd.y = state.at<float>(1);
    	c.statePre.copyTo(c.statePost);
    	c.errorCovPre.copyTo(c.errorCovPost);
		
        cout << "\n Nueva";
		cout << "\n Posicion x: " << detecInd.x << " y: " << detecInd.y;    		
		detT.posA=detecInd;
		
		for(int ki=0;ki<10;ki++){
            //predict next 10 positions
		    computer_vision::detec detecInd; 
		    state = c.predict();
    		detecInd.width = state.at<float>(4);
    		detecInd.height = state.at<float>(5);
    		detecInd.x = state.at<float>(0);
    		detecInd.y = state.at<float>(1);
    		c.statePre.copyTo(c.statePost);
    		c.errorCovPre.copyTo(c.errorCovPost);
    		cout << "\n Posicion x: " << detecInd.x << " y: " << detecInd.y;    		
			//detT.posSig[ki]=detecInd;
			detT.posSig.push_back(detecInd);
		}
		
		det.array.push_back(detT);
		m++;
		
	}
    	detecciones.publish(det);  
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "detec_vehiculos");
	ros::NodeHandle nh("~");
	ros::Subscriber camara_sub = nh.subscribe("/app/camera/rgb/image_raw", 1, camaraRGBCallback);
	
    detecciones = nh.advertise<computer_vision::detecArray>("/detecciones",1);//

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	
	ros::Rate loop_rate(RATE_HZ);

	String cascadaFile = "/home/israel/EK_AutoNOMOS_Sim/src/computer_vision/src/cascade.xml"; 
    String svmFile = "/home/israel/EK_AutoNOMOS_Sim/src/computer_vision/src/my_svmML4C.yml"; //my_svmML3C.yml
    String svmFile2 = "/home/israel/EK_AutoNOMOS_Sim/src/computer_vision/src/my_svmML4L.yml"; //my_svmML3L.yml

    cout << "Iniciando detector..." << endl;
	
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
   
	//Se carga la máquina de soporte vectorial
    
    svm = StatModel::load<SVM>(svmFile);
    svm2 = StatModel::load<SVM>(svmFile2);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    std::cout << "\nCargar SVM Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    begin = std::chrono::steady_clock::now();
	
	//Se carga el filtro de cascada
    carC.load(cascadaFile);
    end = std::chrono::steady_clock::now();

    std::cout << "\nCargar cascada: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

	std::chrono::steady_clock::time_point lastPrediction;
    std::chrono::steady_clock::time_point PrediccionActual;

	//Vectores para guardar los objetos detectados, cuando fue el último frame en que aparecio, y sus filtros de kalman asociados
    vector<Rect> objetos;
    vector<int> ultimoFrame;
    vector<KalmanFilter> filtrosK;
    cout << "\n svm cargada";
	
    int cont = 5;
	int i=0;
	 
	ros::Duration(1).sleep();

	while(nh.ok()){
		ros::spinOnce();               
		current_time = ros::Time::now();
		std::cout<<"\n Inicio "<<current_time;
		std::chrono::steady_clock::time_point begin2 = std::chrono::steady_clock::now();
		
		Mat imgF;
		int numberO = (int)objetos.size();
		vector<Rect> detections;
		int numberD;
		
		begin = std::chrono::steady_clock::now();
        detectionStep(&cont, numberO, &objetos, &imgF, &filtrosK, &detections, &numberD, i);
		end = std::chrono::steady_clock::now();

		std::cout << "\nTime total deteccion = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
		begin = std::chrono::steady_clock::now();

        trackingStep(numberD, &objetos, &detections, &ultimoFrame, &filtrosK, &lastPrediction, &PrediccionActual, &numberO, i, &imgF);
		
        publishMessage(numberO, &filtrosK);

        lastPrediction = PrediccionActual;
		
		end = std::chrono::steady_clock::now();
		std::cout << "\nTiempo total kalman = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

		std::chrono::steady_clock::time_point end2 = std::chrono::steady_clock::now();
		std::cout << "\nTiempo por frame = " << std::chrono::duration_cast<std::chrono::milliseconds>(end2 - begin2).count() << "[ms]" << std::endl;

		std::cout<<"\n Fin"<<ros::Time::now();
		last_time = current_time;
		loop_rate.sleep();
		i++; //contador de frames
	}
	
}
