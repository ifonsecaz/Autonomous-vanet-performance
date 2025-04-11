#include <cv_bridge/cv_bridge.h> 
#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/ml/ml.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <time.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <stdint.h>
#include <vector>
#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>
#include <unistd.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <boost/foreach.hpp>

#include "prueba/detec.h"
#include "prueba/detecTiempos.h"
#include "prueba/detecArray.h"

using namespace cv;
using namespace cv::ml;
using namespace std;

cv::Mat img;
int RATE_HZ = 5;

ros::Publisher detec_publisher;
ros::Publisher detec_publisherk;

cv_bridge::CvImage img_bridge;

void camaraRGBCallback(const sensor_msgs::Image& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    
    img = cv_ptr->image.clone();
    std::cout << img.channels();
    //std::cout<<"\n Imagen cargada";
}

void test_trained_detectorIm(Ptr<SVM> svm,CascadeClassifier carC, Mat img, int cont, Mat* imgF,int* numberDetec,vector<Rect>* detections)
{
    ros::NodeHandle nh("~");
    detec_publisher = nh.advertise<sensor_msgs::Image>("/detec",1);
    Mat img0, img1;
    Mat imgZ = Mat(480, 640, CV_64F, 0.0);

	//ROI
	//Mat img0 = Mat(415, 640, CV_64F, 0.0);
	//img(Rect(0, 0, 640, 415).copyTo(img0);
    //resize(img, img0, Size(640, 480), 0, 0, cv::INTER_AREA); //No
    //std::cout<<"\n color";
    //cvtColor(img, img1, COLOR_BGR2GRAY);
    img.convertTo(img1,CV_8UC3);

    vector< Rect > detections2;
    vector< Rect > detectionsAux;
    Mat imgAux;
    Mat imgClas;
    Mat imgClas2;

    HOGDescriptor hog;
    hog.winSize = Size(64, 64);
    hog.blockSize = Size(16, 16);
    hog.cellSize = Size(8, 8);
    hog.blockStride = Size(8, 8);
    hog.nbins = 9;
    hog.derivAperture = 1;
    hog.winSigma = 4;
    //hog.histogramNormType = 0;
    hog.L2HysThreshold = 2.0000000000000001e-01;
    hog.gammaCorrection = 1;
    hog.nlevels = 64;
    hog.signedGradient = 0;

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    carC.detectMultiScale(img, detections2, 1.1, 2, 0, Size(50, 50), Size(200, 200));
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    //std::cout << "Time detect cascade " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
    //*detections = detections2;
    //*numberDetec = (int) detections2.size();
    //cout << "\n number of detections: " << *numberDetec;
    
    std::cout << "\n number of detections: " << (int)detections2.size();
    int aux = 0;
    for (size_t j = 0; j < detections2.size(); j++)
    {
        //Scalar color = Scalar(0, foundWeights[j] * foundWeights[j] * 200, 0);
        imgAux=Mat(detections2[j].width, detections2[j].height, CV_64F, 0.0); //CV_64F
        img(Rect(detections2[j].x, detections2[j].y, detections2[j].width, detections2[j].height)).copyTo(imgAux(Rect(0, 0, detections2[j].width, detections2[j].height)));

        resize(imgAux, imgClas, Size(64, 64), 0, 0, cv::INTER_AREA);
        vector< float > descriptors;
        imgClas.convertTo(imgClas2, CV_8UC3);
        hog.compute(imgClas2, descriptors, Size(0, 0), Size(0, 0));
        //Mat HOGFeat_test(1, descriptors.size(), CV_32FC1); //si no funciona o con datos a ml
        //for (int i = 0; i < descriptors.size(); i++)
        //    HOGFeat_test.at<float>(0, i) = descriptors.at(i);
        float result = svm->predict(descriptors);

        //float result = svm->predict(HOGFeat_test);
        std::cout << "\n result " << result;
        if (result > 0.3) { //-0.2 o -0.15 ok
            aux++;
            detectionsAux.push_back(detections2[j]);
            rectangle(img1, detections2[j], 200, img1.cols / 400 + 1);
            img1(Rect(detections2[j].x, detections2[j].y, detections2[j].width, detections2[j].height)).copyTo(imgZ(Rect(detections2[j].x, detections2[j].y, detections2[j].width, detections2[j].height)));
        }
    }

    *numberDetec = aux;

    *imgF =  img1; ///
    *detections = detectionsAux;


    //imshow("detec",img1);
    String a;
    //mask = np.zeros(img.shape, np.uint8)
    //imgZ[y:y + h, x : x + w] = img[y:y + h, x : x + w]

    a = "test" + std::to_string(cont) + ".jpg";
    string path = "/home/israel/Documents/k/";
    String b = path + "/recorte" + std::to_string(cont) + ".jpg";

    imwrite(a, img1);
    imwrite(b, imgZ);
    //waitKey(1);
    
    sensor_msgs::Image img_msg;
    std_msgs::Header header; 
    //header.seq = counter; 
    header.stamp = ros::Time::now(); 
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC1, img1);
    img_bridge.toImageMsg(img_msg); 
    detec_publisher.publish(img_msg); 
    //detec_publisher.publish(bridge_.cvToImgMsg(img1,"gray"));
}

void test_region_detectorIm(Ptr<SVM> svm, CascadeClassifier carC, Mat img, int cont, int extra, int x, int y, int width, int height, Mat* imgF, bool* encontro, Rect* detections)
{
    ros::NodeHandle nh("~");
    detec_publisher = nh.advertise<sensor_msgs::Image>("/detec",1);
    
    Mat img0, img1;
    Mat imgZ = Mat(480, 640, CV_64F, 0.0);
    *encontro= false;

    //resize(img, img0, Size(640, 480), 0, 0, cv::INTER_AREA); //no se incluye en el nodo de ROS
    if (img.channels() == 3) {
        cvtColor(img, img1, COLOR_BGR2GRAY);
    }
    else {
        img1 = img;
    }

    int x1, y1,width1,height1;
    if ((x + width +extra)> 640) {
        width1 = 640 - x + extra;
        x1 = x - extra;
    }
    else {
        if ((x-extra)<0) {
            x1 = 0;
            width1 = width + extra + x;
        }
        else {
            x1 = x - extra;
            width1 = width + 2 * extra;
        }

    }
    if (y + height > 480) { //Si se usa ROI cambiar a 415
        height1 = 480 - y + extra;
        y1 = y - extra;
    }
    else {
        if ((y - extra) < 0) {
            y1 = 0;
            height1 = height + extra + y;
        }
        else {
            y1 = y - extra;
            height1 = height + 2 * extra;
        }
    }
    Mat img2 = Mat(height1, width1, CV_64F, 0.0);

    vector< Rect > detections2;
    vector< Rect > detectionsAux;
    Mat imgAux;
    Mat imgClas;
    Mat imgClas2;
    /*
    //opcional
    HOGDescriptor hog;
    hog.winSize = Size(64, 64);
    hog.blockSize = Size(16, 16);
    hog.cellSize = Size(8, 8);
    hog.blockStride = Size(8, 8);
    hog.nbins = 9;
    hog.derivAperture = 1;
    hog.winSigma = 4;
    //hog.histogramNormType = 0;
    hog.L2HysThreshold = 2.0000000000000001e-01;
    hog.gammaCorrection = 1;
    hog.nlevels = 64;
    hog.signedGradient = 0;
    //
    */
    //Recorto la imagen
    img1(Rect(x1,y1,width1,height1)).copyTo(img2);


    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    carC.detectMultiScale(img2, detections2, 1.1, 2, 0, Size(50, 50), Size(width1,height1));
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    //std::cout << "Time detect cascade " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
    //*detections = detections2;
    //*numberDetec = (int) detections2.size();
    //cout << "\n number of detections: " << *numberDetec;

    cout << "\n number of detections: " << (int)detections2.size();
    if ((int(detections2.size() > 0))) {
        *encontro = true;
        //Sin Validacion gana 50 ms aproximadamente
        //Con hog 250-300 ms, sin 200-250 ms
        /*
        int aux = 0;
        for (size_t j = 0; j < detections2.size(); j++)
        {
            //opcional
            //Scalar color = Scalar(0, foundWeights[j] * foundWeights[j] * 200, 0);
            imgAux = Mat(detections2[j].width, detections2[j].height, CV_64F, 0.0); //CV_64F
            img2(Rect(detections2[j].x, detections2[j].y, detections2[j].width, detections2[j].height)).copyTo(imgAux(Rect(0, 0, detections2[j].width, detections2[j].height)));

            resize(imgAux, imgClas, Size(64, 64), 0, 0, cv::INTER_AREA);
            vector< float > descriptors;
            imgClas.convertTo(imgClas2, CV_8UC3);
            hog.compute(imgClas2, descriptors, Size(0, 0), Size(0, 0));
            //Mat HOGFeat_test(1, descriptors.size(), CV_32FC1); //si no funciona o con datos a ml
            //for (int i = 0; i < descriptors.size(); i++)
            //    HOGFeat_test.at<float>(0, i) = descriptors.at(i);
            float result = svm->predict(descriptors);

            //float result = svm->predict(HOGFeat_test);
            cout << "\n result " << result;
            if (result > 0.3) { //-0.2 o -0.15 ok
                aux++;
                //detectionsAux.push_back(detections2[j]);
                rectangle(img1, Rect(x1 + detections2[j].x, y1 + detections2[j].y, detections2[j].width, detections2[j].height), 200, img1.cols / 400 + 1);
                //img1(Rect(x1 + detections2[j].x, y1 + detections2[j].y, detections2[j].width, detections2[j].height)).copyTo(imgZ(Rect(x1 + detections2[j].x, y1 + detections2[j].y, detections2[j].width, detections2[j].height)));
            }
            //

        }
        */
        rectangle(img1, Rect(x1 + detections2[0].x, y1 + detections2[0].y, detections2[0].width, detections2[0].height), 200, img1.cols / 400 + 1); //Quitar si se incluye hog
        
        *imgF = img1; ///
        *detections = Rect(x1 + detections2[0].x, y1 + detections2[0].y, detections2[0].width, detections2[0].height);
        //*detections = detections2[0];

        //imshow("detec",img1);
        String a;
        //mask = np.zeros(img.shape, np.uint8)
        //imgZ[y:y + h, x : x + w] = img[y:y + h, x : x + w]

        a = "test" + std::to_string(cont) + ".jpg";
        string path = "/home/israel/Documents/k/";
        String b = path + "/recorte" + std::to_string(cont) + ".jpg";
        
        sensor_msgs::Image img_msg;
	std_msgs::Header header; 
	//header.seq = counter; 
	header.stamp = ros::Time::now(); 
	img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC1, img1);
        img_bridge.toImageMsg(img_msg); 
	detec_publisher.publish(img_msg); 
        //detec_publisher.publish(bridge_.cvToImgMsg(img1,"gray"));
        imwrite(a, img1);
        imwrite(b, imgZ);
        //waitKey(1);
    }
    else {
        *imgF = img1;
        //*detections = ;
    }
    

}

void kalman(KalmanFilter kf, Rect detections, Mat img,int i,bool found) {
    ros::NodeHandle nh("~");
    detec_publisherk = nh.advertise<sensor_msgs::Image>("/deteck",1);
    //KalmanFilter(int  	dynamParams,        int  	measureParams,        int  	controlParams = 0,        int  	type = CV_32F)
    //correct
    //predict
    //Mat prediction = KF.predict();
    //Calculo de la velocidad
    Mat meas(4, 1, CV_32F);
    Mat state(6, 1, CV_32F);
    double dT = 1/38;
    //dT=1/fps*algo

    kf.transitionMatrix.at<float>(2) = dT;
    kf.transitionMatrix.at<float>(9) = dT;

    state = kf.predict();

    //Creo un rectangulo
    Rect predRect;
    predRect.width = state.at<float>(4);
    predRect.height = state.at<float>(5);
    predRect.x = state.at<float>(0);
    //predRect.x = state.at<float>(0) - predRect.width / 2;
    predRect.y = state.at<float>(1);
    //predRect.y = state.at<float>(1) - predRect.height / 2;

    if (found) {

        meas.at<float>(0) = detections.x;
        meas.at<float>(1) = detections.y;
        meas.at<float>(2) = detections.width;
        meas.at<float>(3) = detections.height;

        kf.correct(meas);
    }
    cv::rectangle(img, predRect, CV_RGB(255, 0, 0), 2);
    
    
    
    string path = "/home/israel/Documents/k/kalman/";
    String b = path + "/res" + std::to_string(i) + ".jpg";
    
    
    sensor_msgs::Image img_msg;
    std_msgs::Header header; 
    //header.seq = counter; 
    header.stamp = ros::Time::now(); 
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC1, img);
    img_bridge.toImageMsg(img_msg); 
    detec_publisherk.publish(img_msg);  
    //detec_publisherk.publish(bridge_.cvToImgMsg(img,"gray"));
    imwrite(b, img);
    //waitKey(1);

    //Mismo tipo 
    //KF.correct(measurement);
}

//Metodo preliminar que procesa los primeros dos frames
//Solo entra la primera vez
void preparacionKalman(Rect detections, KalmanFilter* kalman) {
    //transition matrix, control matrix, measurement matrix, state, cov7
    //control matrix solo si hay control
    //KalmanFilter(int  	dynamParams,        int  	measureParams,        int  	controlParams = 0,        int  	type = CV_32F)
    //Se controla la posicion y velocidad sobre una imagen 2D, sin control
    // 4 parametros, 2 de medicion
    //1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1
    int stateSize = 6; //[x, y, v_x, v_y, w, h] posiciones del centro, velocidades, y tamano del rectangulo dibujado
    int measSize = 4; //[z_x,z_y,z_w,z_h]  se mide la posicion y tamano del rectangulo
    //measure solo si se detecta el coche en el frame actual para la correccion, si no solo prediccion
    int contrSize = 0;

    //solo recibe un auto
    //falta considerar crear multiples filtros por cada coche detectado
    // Como identificar entre frames que corresponden al mismo auto
    // sacar colores rectangulo detectado y guardarlos?, problema con las luces y sombras
    // otro metodo intermedio despues de las detecciones que analice cuantos fueron encontrados y se asegure coincidan con los ya detectados, si no llama a este metodo y crea un nuevo filtro para el nuevo auto
    // se deben guardar en un vector los autos localizados, si tras x frames no se vuelve a encontrar descartarlo
    //for (size_t j = 0; j < detections.size(); j++)
    //{

    //}

    KalmanFilter kf(stateSize, measSize, contrSize);

    //Matriz estados
    Mat state(stateSize, 1, CV_32F);
    //Matriz mediciones
    Mat meas(measSize, 1, CV_32F);

    setIdentity(kf.transitionMatrix);


    //Matriz de medicion es 4,6
    //  x  y  vx vy w  h
    //  1  0  0  0  0  0  
    //  0  1  0  0  0  0  
    //  0  0  0  0  1  0  
    //  0  0  0  0  0  1
    // se obtienen las mediciones de x y w h, no las velocidades
    kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, CV_32F);
    kf.measurementMatrix.at<float>(0) = 1.0f;
    kf.measurementMatrix.at<float>(7) = 1.0f;
    kf.measurementMatrix.at<float>(16) = 1.0f;
    kf.measurementMatrix.at<float>(23) = 1.0f;

    //matriz de covarianzas
    // Si se asumen no estan correlacionadas 
    // [ Ex   0   0     0     0    0  ]
    // [ 0    Ey  0     0     0    0  ]
    // [ 0    0   Ev_x  0     0    0  ]
    // [ 0    0   0     Ev_y  0    0  ]
    // [ 0    0   0     0     Ew   0  ]
    // [ 0    0   0     0     0    Eh ]
    kf.processNoiseCov.at<float>(0) = 1e-2;
    kf.processNoiseCov.at<float>(7) = 1e-2;
    kf.processNoiseCov.at<float>(14) = 5.0f;
    kf.processNoiseCov.at<float>(21) = 5.0f;
    kf.processNoiseCov.at<float>(28) = 1e-2;
    kf.processNoiseCov.at<float>(35) = 1e-2;

    setIdentity(kf.measurementNoiseCov, Scalar(1e-5));
    //setIdentity(kf.measurementMatrix);
    //setIdentity(kf.processNoiseCov, Scalar::all(1e-5));
    //setIdentity(kf.errorCovPost, Scalar::all(1));

    //x y no correlacionadas, velocidades si
    // [ Ex   0   xvx   0     0    0  ]
    // [ 0    Ey  0     0     0    0  ]
    // [ xvx  0   Ev_x  yvy   0    0  ]
    // [ 0    yvy 0     Ev_y  0    0  ]
    // [ 0    0   0     0     Ew   0  ]
    // [ 0    0   0     0     0    Eh ]

    //Se saca la posicion del auto detectado
    //primera medicion
    kf.errorCovPre.at<float>(0) = 1; // px
    kf.errorCovPre.at<float>(7) = 1; // px
    kf.errorCovPre.at<float>(14) = 1;
    kf.errorCovPre.at<float>(21) = 1;
    kf.errorCovPre.at<float>(28) = 1; // px
    kf.errorCovPre.at<float>(35) = 1; // px

    state.at<float>(0) = detections.x;
    state.at<float>(1) = detections.y;
    state.at<float>(2) = 0;
    state.at<float>(3) = 0;
    state.at<float>(4) = detections.width;
    state.at<float>(5) = detections.height;

    kf.statePost = state;

    *kalman = kf;

}


int main(int argc, char** argv){
  ros::init(argc, argv, "detec_vehiculos");

  ros::NodeHandle nh("~");
  ros::Subscriber camara_sub = nh.subscribe("/app/camera/rgb/image_raw", 10, camaraRGBCallback);
  
  ros::Publisher detecciones = nh.advertise<prueba::detecArray>("/detecciones",1);;//

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  
  ros::Rate loop_rate(RATE_HZ);

  String svmFile = "/home/israel/EK_AutoNOMOS_Sim/src/prueba/src/my_svmDecente.yml";
  String obj_det_filename = "/home/israel/EK_AutoNOMOS_Sim/src/prueba/src/cascade.xml";
  
  //Hog features
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  Ptr<SVM> svm;
  svm = StatModel::load<SVM>(svmFile);
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  std::cout << "\nCargar SVM Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
  
  //Filtro de cascada
  begin = std::chrono::steady_clock::now();

  CascadeClassifier carC;
  carC.load(obj_det_filename);
  end = std::chrono::steady_clock::now();

  std::cout << "\nCargar cascada: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
	
  vector<Rect> objetos;
  vector<int> ultimoFrame;
  vector<KalmanFilter> kalmans;
  int cont = 5;

  int i=0;
  std::chrono::steady_clock::time_point begin2;
  std::chrono::steady_clock::time_point end2;

  ros::Duration(1).sleep();
  	
	
  while(nh.ok()){
    ros::spinOnce();               
    current_time = ros::Time::now();
	std::cout<<"\n Inicio "<<current_time;
    begin2 = std::chrono::steady_clock::now();
	
	Mat img2;
    int numberO = (int)objetos.size();
    vector<Rect> detections;
    int numberD;
	
    begin = std::chrono::steady_clock::now();
	if (cont == 5) { //cada 5 frames deteccion completa
	  std::cout<<"\n Inicppp";
		test_trained_detectorIm(svm, carC, img, i, &img2, &numberD, &detections); //cargar svm7
		cont = 1;
	}
	else {
		numberD = 0;
		for (size_t j = 0; j < numberO; j++) {
			//tamano region,x(esquina),y,width,height
			//detections regresa la nueva pos
			//bool si se detecto
			//SOlo busca en la region, pero dibuja el rectangulo en la imagen completa
			Rect detec;
			bool encontro;
			test_region_detectorIm(svm, carC, img, i, 10,objetos[j].x,objetos[j].y,objetos[j].width,objetos[j].height,&img2,&encontro, &detec); //Busca en una region 20 pixeles mas grande
			img = img2;
			if (encontro) {
				detections.push_back(detec);
				numberD++;
			}
		}
		cont++;
	}
	end = std::chrono::steady_clock::now();

	std::cout << "\nTime total deteccion = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
	begin = std::chrono::steady_clock::now();

	for (size_t j = 0; j < numberD; j++) {
            //Varianza, +-20 pixeles es el mismo objeto
            int k = 0;
            bool resp = false;
            while (k < objetos.size() & !resp) {
                //localizo el centro
                //Identificar si es el mismo vehiculo entre dos frames
                if (abs((detections[j].x+(detections[j].width/2))-(objetos[k].x+(objetos[k].width/2)))<=20 && abs((detections[j].y+(detections[j].height/2)) - (objetos[k].y+(objetos[k].height/2))) <= 20) {
                    resp = true; //Ya esta
                }
                k++;
            }
            if (!resp) {
                objetos.push_back(detections[j]); //nuevo
                ultimoFrame.push_back(i);
                KalmanFilter a;
                preparacionKalman(detections[j], &a);
                kalmans.push_back(a);
            }
            else
            {
                objetos[k - 1] = detections[j]; //Predice
                ultimoFrame[k - 1] = i;
            }
        }
		cout << "\nTotal objetos: " << objetos.size();
        for (size_t m = 0; m < objetos.size(); m++) {
            cout << "\n Posicion x: " << objetos[m].x << " y: " << objetos[m].y << " ultimo frame: " << ultimoFrame[m];
        }
        int m = 0;
        while (m < numberO) {
            if (i == ultimoFrame[m]) {
                //La i se quita, solo para guardar las imagenes
                //Considerar ultimo frame para ya no calcularlo ej i-5
                kalman(kalmans[m], objetos[m], img2,i,true);
            }
            else {
                if (i - ultimoFrame[m] < 5) {
                    kalman(kalmans[m], objetos[m], img2, i, false);
                }
                else
                {
                    kalmans.erase(kalmans.begin()+m);
                    objetos.erase(objetos.begin() + m);
                    ultimoFrame.erase(ultimoFrame.begin() + m);
                    m--;
                    numberO--;
                }
            }
            m++;
        }
        
        prueba::detecArray det; //

	det.numDetec=numberO;
	m=0;
	while(m<numberO){
	        prueba::detecTiempos detT;
		KalmanFilter c;
    		c.controlMatrix = kalmans[m].controlMatrix.clone();
		c.errorCovPost = kalmans[m].errorCovPost.clone();
		c.errorCovPre = kalmans[m].errorCovPre.clone();
		c.gain = kalmans[m].gain.clone();
		c.measurementMatrix = kalmans[m].measurementMatrix.clone();
		c.measurementNoiseCov = kalmans[m].measurementNoiseCov.clone();
    		c.processNoiseCov = kalmans[m].processNoiseCov.clone();
		c.statePost = kalmans[m].statePost.clone();
		c.statePre = kalmans[m].statePre.clone();
		c.transitionMatrix = kalmans[m].transitionMatrix.clone();
		c.temp1 = kalmans[m].temp1.clone();
		c.temp2 = kalmans[m].temp2.clone();
		c.temp3 = kalmans[m].temp3.clone();
		c.temp4 = kalmans[m].temp4.clone();
		c.temp5 = kalmans[m].temp5.clone();
		
		prueba::detec detecInd; 
		Mat state(6, 1, CV_32F);
		state = c.predict();
    		detecInd.width = state.at<float>(4);
    		detecInd.height = state.at<float>(5);
    		detecInd.x = state.at<float>(0);
    		detecInd.y = state.at<float>(1);
    		c.statePre.copyTo(c.statePost);
    		c.errorCovPre.copyTo(c.errorCovPost);
    		
		detT.posA=detecInd;
		
		for(int ki=0;ki<10;ki++){
		        prueba::detec detecInd; 
			state = c.predict();
    			detecInd.width = state.at<float>(4);
    			detecInd.height = state.at<float>(5);
    			detecInd.x = state.at<float>(0);
    			detecInd.y = state.at<float>(1);
    			c.statePre.copyTo(c.statePost);
    			c.errorCovPre.copyTo(c.errorCovPost);
    		
			//detT.posSig[ki]=detecInd;
			detT.posSig.push_back(detecInd);
		}
		det.array.push_back(detT);
	}


    	detecciones.publish(det);  
    
    
    
	end = std::chrono::steady_clock::now();
    std::cout << "\nTiempo total kalman = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    end2 = std::chrono::steady_clock::now();
	std::cout << "\nTiempo por frame = " << std::chrono::duration_cast<std::chrono::milliseconds>(end2 - begin2).count() << "[ms]" << std::endl;

	std::cout<<"\n Fin"<<ros::Time::now();
	last_time = current_time;
    loop_rate.sleep();
	i++; //contador de frames
  }
}
