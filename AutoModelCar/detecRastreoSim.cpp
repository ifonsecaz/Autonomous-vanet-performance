#include <cv_bridge/cv_bridge.h> 
#include "opencv2/imgproc/imgproc.hpp"
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
#include <boost/foreach.hpp>

using namespace cv;
using namespace cv::ml;
using namespace std;

//Define la taza a la que se publica las detecciones, qué altura considerar
//img guarda la última imagen publicada por la cámara en formato MAT para OpenCV
int RATE_HZ = 5;
int ROI=400;
cv::Mat img= Mat(ROI, 640, CV_64F, 0.0);

//Se definen los publicadores para usarlos dentro de las funciones
ros::Publisher detec_publisher; //Publica la imagen con las detecciones
ros::Publisher detec_publisherk; //Publica la imagen con las predicciones 

cv_bridge::CvImage img_bridge;

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

/*
Método que realiza la deteccion sobre la imagen completa
Recibe las máquinas de soporte vectorial y el filtro de cascada, el frame actual y el número del frame
Devuelve la imagen con los rectángulos marcados de las detecciones, el vector de rectángulos y el número
Según se use una sola o las dos máquinas de soporte vectorial, se debe descomentar la función correspondiente y cambiar en result de SVM
*/
void detecVentana(Ptr<SVM> svm, Ptr<SVM> svm2, CascadeClassifier carC, Mat img, int cont, Mat* imgF, int* numberDetec, vector<Rect>* detections)
//void detecVentana(Ptr<SVM> svm,CascadeClassifier carC, Mat img, int cont, Mat* imgF,int* numberDetec,vector<Rect>* detections)
{
	//     /detec es el nombre del tópico para las detecciones, publica un mensaje del tipo sensor_msgs: Image
	ros::NodeHandle nh("~");
    detec_publisher = nh.advertise<sensor_msgs::Image>("/detec",1);
	
    Mat imgGRAY;
    img.convertTo(imgGRAY,CV_8UC3);

    vector< Rect > detectionsCascada;
    vector< Rect > detectionsFinal;
    Mat imgRec;
    Mat imgSVM;
	Mat imgSVMF;
    Mat imgSVMBLUR;

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
	
	//Se realiza la detección con el filtro de cascada sobre la imagen completa, considera un tamaño mínimo de 40x40 pixeles, y máximo de 200x200, va incrementando el tamaño de la ventana por 1.1
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    carC.detectMultiScale(imgGRAY, detectionsCascada, 1.1, 2, 0, Size(40, 40), Size(200, 200));
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    
    cout << "\n number of detections: " << (int)detectionsCascada.size();
    int aux = 0;
	//Cada detección de cascada se evalúa, primero se toma un área alrededor un 20% más grande
    for (size_t j = 0; j < detectionsCascada.size(); j++)
    {
        int width = detectionsCascada[j].width * 0.2;
        int height = detectionsCascada[j].height * 0.2;
		//Revisa que no rebase la imagen 
        if ((detectionsCascada[j].x - (int)(width/2)) > 0 && (detectionsCascada[j].y - (int)(height/2)) > 0 && (detectionsCascada[j].x + detectionsCascada[j].width + (int)(width/2)) < 640 && (detectionsCascada[j].y + detectionsCascada[j].height + (int)(height/2))<400) {
            imgRec = Mat(detectionsCascada[j].width+width, detectionsCascada[j].height+height, CV_64F, 0.0); //CV_64F 
            imgGRAY(Rect(detectionsCascada[j].x-(int)(width/2), detectionsCascada[j].y-(int)(height/2), detectionsCascada[j].width+width, detectionsCascada[j].height+height)).copyTo(imgRec(Rect(0, 0, detectionsCascada[j].width+width, detectionsCascada[j].height+height)));
        }
        else {
            imgRec = Mat(detectionsCascada[j].width, detectionsCascada[j].height, CV_64F, 0.0); //CV_64F 
            imgGRAY(Rect(detectionsCascada[j].x, detectionsCascada[j].y, detectionsCascada[j].width, detectionsCascada[j].height)).copyTo(imgRec(Rect(0, 0, detectionsCascada[j].width, detectionsCascada[j].height)));
        }
		//Para la predicción, la imagen debe ser de 64x64
        resize(imgRec, imgSVM, Size(64, 64), 0, 0, cv::INTER_AREA);
		
		//Se aplica un filtro gaussiano, igual se puede probar con motionblur, se da el formato correcto y se extraen sus características
        //GaussianBlur(imgSVM, imgSVMBLUR, Size(5, 5), 0);
        vector< float > descriptorsSVM;
        imgSVM.convertTo(imgSVMF, CV_8UC3);
        hog.compute(imgSVMF, descriptorsSVM, Size(0, 0), Size(0, 0));
        
		//Se realiza la predicción con SVM
		//Se debe cambiar para usar solo una máquina 
        float result1 = svm->predict(descriptorsSVM);
        float result;
        if (result1 < 0) { //OR
            result = svm2->predict(descriptorsSVM);
        }
        else {
            result = result1;
        }
       
        cout << "\n result " << result;
		//Agrega las detecciones clasificadas como +1, es decir positivas, se guardan duplicadas para el método groupRectangles
        if (result > 0) { 
			aux++;
            detectionsFinal.push_back(detectionsCascada[j]);
            detectionsFinal.push_back(detectionsCascada[j]);
        }
    }
    //Resultados pobres
	//El detector de cascada falla al detectar autos que van apareciendo por las esquinas, se probó a tomarlas para hacer la clasificación con SVM
    //Esquina inferior derecha
	//Se recorta la esquina y se le da formato, se calculan sus descriptores y se realiza la predicción
	Mat imgEsquina = Mat(130, 130, CV_64F, 0.0); //CV_64F
    imgGRAY(Rect(510, 260, 130, 130)).copyTo(imgEsquina(Rect(0, 0, 130, 130)));


    resize(imgEsquina, imgSVM, Size(64, 64), 0, 0, cv::INTER_AREA);

    //GaussianBlur(imgSVM, imgSVMBLUR, Size(5, 5), 0);
    vector< float > descriptorsSVM;
    imgSVM.convertTo(imgSVMF, CV_8UC3);
    hog.compute(imgSVMF, descriptorsSVM, Size(0, 0), Size(0, 0));
    float result1 = svm->predict(descriptorsSVM); //usar svm2
    float result;
    if (result1 < 0) { 
        result = svm2->predict(descriptorsSVM);
    }
    else {
        result = result1;
    }
    cout << "\n" << result;
    if (result > 0) { //0.0 simulador, caso real 0.3
        aux++;
        detectionsFinal.push_back(Rect(510,260,130,130));
        detectionsFinal.push_back(Rect(510,260,130,130));
    }

    //Esquina inferior izquierda
    imgGRAY(Rect(0,260, 130, 130)).copyTo(imgEsquina(Rect(0, 0, 130, 130)));

    resize(imgEsquina, imgSVM, Size(64, 64), 0, 0, cv::INTER_AREA);

    //GaussianBlur(imgSVM, imgSVMBLUR, Size(5, 5), 0);
    imgSVM.convertTo(imgSVMF, CV_8UC3);
    hog.compute(imgSVMF, descriptorsSVM, Size(0, 0), Size(0, 0));
    result1 = svm->predict(descriptorsSVM); 
    result;
    if (result1 < 0) {
        result = svm2->predict(descriptorsSVM);
    }
    else {
        result = result1;
    }
    cout <<"\n"<< result;
    if (result > 0) { 
        aux++;
        detectionsFinal.push_back(Rect( 0,260, 130, 130));
        detectionsFinal.push_back(Rect(0, 260, 130, 130));
    }
    
	//Se calcula el promedio de los rectángulos detectados que se sobreponen
    groupRectangles(detectionsFinal, 1, 0.60); //groupThreshold numero minimo posible de rectangulos para retenerlo, eps cuanto deben sobreponerse, cuanto más cercano es a 0 no se agrupan
    aux = detectionsFinal.size();
    for (int j = 0; j < detectionsFinal.size(); j++) {
        rectangle(imgGRAY, detectionsFinal[j], 200, imgGRAY.cols / 400 + 1);
        rectangle(imgGRAY, Rect(detectionsFinal[j].x + (detectionsFinal[j].width / 2)-2, detectionsFinal[j].y + (detectionsFinal[j].height / 2)-2, 4, 4), 250, imgGRAY.cols / 400 + 1);
    }

	//Se devuelven los valores y se guarda la imagen con las detecciones
    *numberDetec = aux;

    *imgF =  imgGRAY; ///
    *detections = detectionsFinal;

    String a;

    a = "test" + std::to_string(cont) + ".jpg";

    imwrite(a, imgGRAY);
	
	//Se convierte la imagen a sensor_msgs y se publica
	sensor_msgs::Image img_msg;
    std_msgs::Header header; 
    //header.seq = counter; 
    header.stamp = ros::Time::now(); 
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC1, imgGRAY);
    img_bridge.toImageMsg(img_msg); 
    detec_publisher.publish(img_msg); 
}

/*
Método para realizar detecciones sobre una región, la cual es predicha por kalman
La detección se realiza solo con el clasificador de cascada, o se puede descomentar la función para usar también las máquinas SVM
A partir de la imagen toma una pequeña área más grande, devuelve la imagen, un booleano indicando si lo encontró y la detección
*/
//void detecRegion(Ptr<SVM> svm, Ptr<SVM> svm2, CascadeClassifier carC, Mat img, int cont, int extra, int x, int y, int width, int height, Mat* imgF, bool* encontro, Rect* detections)
void detecRegion(CascadeClassifier carC, Mat img, int cont, int x, int y, int width, int height, Mat* imgF, bool* encontro, Rect* detections)
{
    ros::NodeHandle nh("~");
    detec_publisher = nh.advertise<sensor_msgs::Image>("/detec",1);
	
	Mat imgGRAY;
    *encontro= false;
	
	//Recibe la imagen completa y según la predicción recorta una parte, evalua si puede tomar una ventana hasta un 80% más grande
    if (img.channels() == 3) {
        cvtColor(img, imgGRAY, COLOR_BGR2GRAY);
    }
    else {
        imgGRAY = img;
    }

if(x>0&&x<640&&y>0&&y<480&&width>0&&height>0){
    int x1, y1,width1,height1;
    int extra = (int)0.8 * width;
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
    if (y + height + extra> 480) {
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
    Mat imgROI = Mat(height1, width1, CV_64F, 0.0);

    vector< Rect > detectionsCascada;
    vector< Rect > detectionsFinal;
    Mat imgAux;
	//Mat imgROI;
    Mat imgSVM;
    Mat imgSVMBLUR;
	Mat imgSVMF;
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
    */
    //Recorto la imagen
    imgGRAY(Rect(x1,y1,width1,height1)).copyTo(imgROI);

	//Se realiza la detección de cascada
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    carC.detectMultiScale(imgROI, detectionsCascada, 1.05, 1, 0, Size(width*0.4, height*0.4), Size(width1, height1)); //Dado tamano segun prediccion
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    //std::cout << "Time detect cascade " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

	//En caso de múltiples detecciones, se quiere tomar el promedio de las detecciones con groupRectangles, por ello se duplican
    vector<Rect>detectionDup;
    detectionDup = detectionsCascada;
    detectionsCascada.insert(detectionsCascada.end(), detectionDup.begin(), detectionDup.end());

    cout << "\n number of detections: " << (int)detectionsCascada.size();
    if ((int(detectionsCascada.size() > 0))) {
        *encontro = true;
		//Descomentar para usar las máquinas de soporte vectorial
        //Sin Validacion gana 50 ms aproximadamente
        //Con hog 250-300 ms, sin 200-250 ms en VisualStudio
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
		//Se agregan los rectángulos sobrepuestos, se dibujan sobre la imagen y se guard
        groupRectangles(detectionsCascada, 1, 0.60);
        rectangle(imgGRAY, Rect(x1 + detectionsCascada[0].x, y1 + detectionsCascada[0].y, detectionsCascada[0].width, detectionsCascada[0].height), 200, imgGRAY.cols / 400 + 1); //Quitar si se incluye hog
        rectangle(imgGRAY, Rect(x1+detectionsCascada[0].x + detectionsCascada[0].width / 2-2, y1+detectionsCascada[0].y + detectionsCascada[0].height / 2-2, 4, 4), 250, imgGRAY.cols / 400 + 1);

        *imgF = imgGRAY; ///
        *detections = Rect(x1 + detectionsCascada[0].x, y1 + detectionsCascada[0].y, detectionsCascada[0].width, detectionsCascada[0].height);

        String a;

        a = "test" + std::to_string(cont) + ".jpg";
		
		//Se publica la imagen
		sensor_msgs::Image img_msg;
		std_msgs::Header header; 
		//header.seq = counter; 
		header.stamp = ros::Time::now(); 
		img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC1, imgGRAY);
        img_bridge.toImageMsg(img_msg); 
		detec_publisher.publish(img_msg); 
    }
    else {
		//Devuelve la imagen original si no hubo detecciones
        *imgF = imgGRAY;
    }
else {
		//Devuelve la imagen original si no hubo detecciones
	String a = "/home/israel/Documents/k/test" + std::to_string(cont) + ".jpg";
        *imgF = imgGRAY;
        imwrite(a, imgGRAY);	
    }
}

/*
Filtro de kalman
Realiza la predicción y corrección en caso de que hubiera detecciones, hace la actualización
Guarda una imagen con la predicción
Recibe el filtro, la deteccion, la imagen, un contador, y un booleano para indicar si la detección se encontró en el frame actual 
Devuelve el filtro actualizado
*/
void kalman(KalmanFilter kf, Rect detections, Mat img,int i,bool found, std::chrono::steady_clock::time_point lastP, std::chrono::steady_clock::time_point Pactual,KalmanFilter* act) {
	//     /pred Nombre del tópico con el que se publica la imagen con la predicción
	ros::NodeHandle nh("~");
    detec_publisherk = nh.advertise<sensor_msgs::Image>("/pred",1);
	
	auto d = std::chrono::duration_cast<std::chrono::milliseconds>(Pactual-lastP);
    double dt = std::chrono::duration<double>(d).count();
    kf.transitionMatrix = (Mat_<float>(6, 6) << 1, 0, dt, 0, 0, 0, 0, 1, 0, dt, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1);
	
    //Crea las matrices de estados y de mediciones
    Mat meas(4, 1, CV_32F);
    Mat state(6, 1, CV_32F);
	
	//Primero se realiza la predicción y se guardan los valores como rectángulo
    state = kf.predict();

    Rect predRect;
    predRect.width = state.at<float>(4);
    predRect.height = state.at<float>(5);
    predRect.x = state.at<float>(0);
    //predRect.x = state.at<float>(0) - predRect.width / 2;
    predRect.y = state.at<float>(1);
    //predRect.y = state.at<float>(1) - predRect.height / 2;

	//Si el objeto se detectó en el frame actual, hace la corrección
    if (found) {

        meas.at<float>(0) = detections.x;
        meas.at<float>(1) = detections.y;
        meas.at<float>(2) = detections.width;
        meas.at<float>(3) = detections.height;

        kf.correct(meas);
    }

	//Devuelve el filtro actualizado
    *act = kf;
	//Dibuja la predicción y se guarda la imagen
    if (predRect.x < 640 && predRect.x>0 && predRect.y < 480 && predRect.y>0) {
        cv::rectangle(img, predRect, CV_RGB(255, 0, 0), 2);
        cv::rectangle(img, Rect(predRect.x + predRect.width / 2 - 1, predRect.y + predRect.height / 2 - 1, 2, 2), CV_RGB(1, 0, 0), 2);
    }
    string path = "C:/Users/ifons/source/repos/kalman/kalman/kalman";
    String b = path + "/res" + std::to_string(i) + ".jpg";

    imwrite(b, img);
	
	sensor_msgs::Image img_msg;
    std_msgs::Header header; 
    //header.seq = counter; 
    header.stamp = ros::Time::now(); 
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC1, img);
    img_bridge.toImageMsg(img_msg); 
    detec_publisherk.publish(img_msg);  
}

/*
Método para crear el filtro de kalman
Entra la primera vex que se ve un nuevo objeto
Recibe la detección inicial y devuelve el filtro
*/
void preparacionKalman(Rect detections, KalmanFilter* kalman) {
    //Se prepara la matriz de estados o transiciones y de mediciones
	//Se tienen 6 parámetros pos_x, pos_y, vel_x, vel_y, ancho, altura de la imagen
	//De los parámetros, se miden 4: pos_x, pos_y, ancho y altura
	//El ancho y altura se agregan para dibujar sobre la imagen, además de que pueden ayudar a estimar el tamaño del objeto, como sacar con ayuda del LiDAR o la cámara de profundidad la distancia al objeto de forma más precisa
    //Se controla la posicion y velocidad sobre una imagen 2D, sin control
	//No se agrega por lo tanto una matriz de control
    int stateSize = 6; //[x, y, v_x, v_y, w, h] posiciones del centro, velocidades, y tamano del rectangulo dibujado
    int measSize = 4; //[z_x,z_y,z_w,z_h]  se mide la posicion y tamano del rectangulo

    int contrSize = 0; //Matriz de control
	
	//Se crea el filtro, a continuación se tienen que dar los valores iniciales
    KalmanFilter kf(stateSize, measSize, contrSize);

    //Matriz estados
    Mat state(stateSize, 1, CV_32F);
    //Matriz mediciones
    Mat meas(measSize, 1, CV_32F);
	
    meas.at<float>(0, 0) = detections.x; //Primer punto
    meas.at<float>(1, 0) = detections.y;
    meas.at<float>(2, 0) = detections.width;
    meas.at<float>(3, 0) = detections.height;

	//Se guarda la primera medición como el estado previo y posterior
	//Inicia con una velocidad de cero
    kf.statePre.setTo(0); 
    kf.statePre.at<float>(0, 0) = detections.x;
    kf.statePre.at<float>(1, 0) = detections.y;
    kf.statePre.at<float>(2, 0) = 0;
    kf.statePre.at<float>(3, 0) = 0;
    kf.statePre.at<float>(4, 0) = detections.width;
    kf.statePre.at<float>(5, 0) = detections.height;

    kf.statePost.setTo(0);
    kf.statePost.at<float>(0, 0) = detections.x;
    kf.statePost.at<float>(1, 0) = detections.y;
    kf.statePost.at<float>(2, 0) = 0;
    kf.statePost.at<float>(3, 0) = 0;
    kf.statePost.at<float>(4, 0) = detections.width;
    kf.statePost.at<float>(5, 0) = detections.height;

	//Se define a matriz de transición como
	// 1 0 1 0 0 0
	// 0 1 0 1 0 0
	// 0 0 1 0 0 0
	// 0 0 0 1 0 0
	// 0 0 0 0 1 0
	// 0 0 0 0 0 1
    kf.transitionMatrix = (Mat_<float>(6, 6) << 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1);

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
    /*
    kf.processNoiseCov.at<float>(0) = 1e-2;
    kf.processNoiseCov.at<float>(7) = 1e-2;
    kf.processNoiseCov.at<float>(14) = 5.0f;
    kf.processNoiseCov.at<float>(21) = 5.0f;
    kf.processNoiseCov.at<float>(28) = 1e-2;
    kf.processNoiseCov.at<float>(35) = 1e-2;
    */
    setIdentity(kf.processNoiseCov, Scalar::all(.00005)); //Asigna los mismo valores a la diagonal
	//Se puede ajustar, un valor más grande permite una convergencia más rápida, pero mayor ruido y error
    //Cuanto más bajo lo sigue mejor
	//Con objetos más rápidos o una frecuencia baja se tiene que aumentar

	//De igual forma, se asumen independiente, solo se rellena la diagonal para las matrices de covarianza
    setIdentity(kf.measurementNoiseCov, Scalar(1e-1));

    setIdentity(kf.errorCovPost, Scalar::all(.1));

    //x y no correlacionadas, velocidades si
    // [ Ex   0   xvx   0     0    0  ]
    // [ 0    Ey  0     0     0    0  ]
    // [ xvx  0   Ev_x  yvy   0    0  ]
    // [ 0    yvy 0     Ev_y  0    0  ]
    // [ 0    0   0     0     Ew   0  ]
    // [ 0    0   0     0     0    Eh ]

    *kalman = kf;

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "detec_vehiculos");
	ros::NodeHandle nh("~");
	ros::Subscriber camara_sub = nh.subscribe("/app/camera/rgb/image_raw", 1, camaraRGBCallback);

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	
	ros::Rate loop_rate(RATE_HZ);

	String cascadaFile = "/home/israel/EK_AutoNOMOS_Sim/src/prueba/src/cascade.xml"; 
    String svmFile = "/home/israel/EK_AutoNOMOS_Sim/src/prueba/src/my_svmML4C.yml"; //my_svmML3C.yml
    String svmFile2 = "/home/israel/EK_AutoNOMOS_Sim/src/prueba/src/my_svmML4L.yml"; //my_svmML3L.yml

    cout << "Iniciando detector..." << endl;
	
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
   
	//Se carga la máquina de soporte vectorial
    Ptr<SVM> svm;
    svm = StatModel::load<SVM>(svmFile);
    Ptr<SVM> svm2;
    svm2 = StatModel::load<SVM>(svmFile2);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    std::cout << "\nCargar SVM Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    begin = std::chrono::steady_clock::now();
	
	//Se carga el filtro de cascada
    CascadeClassifier carC;
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
		if (cont == 5) { //cada 5 frames deteccion completa
			//detecVentana(svm, carC, img, i, &imgF, &numberD, &detections); //cargar svm7
			detecVentana(svm, svm2,carC, img, i, &imgF, &numberD, &detections); //cargar svm7
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
				bool encontro=false;
				
				Mat state(6, 1, CV_32F);

				state = filtrosK[j].predict();

				//Creo un rectangulo
				Rect predRect;
				predRect.width = state.at<float>(4);
				predRect.height = state.at<float>(5);
				predRect.x = state.at<float>(0);
				predRect.y = state.at<float>(1);

				if(predRect.x<640&&predRect.x>0&&predRect.y<ROI&&predRect.y>0){
					detecRegion(carC, img, i, predRect.x, predRect.y, predRect.width, predRect.height, &imgF, &encontro, &detec);                    
				}
				if (encontro) {
					detections.push_back(detec);
					numberD++;
				}
				else {
					//Busca en su posicion anterior
					detecRegion(carC, img, i, objetos[j].x, objetos[j].y, objetos[j].width, objetos[j].height, &imgF, &encontro, &detec);
					if (encontro) {
						detections.push_back(detec);
						numberD++;
					}
					else{
						imgF = img;
					}
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
					if (abs((detections[j].x+(detections[j].width/2))-(objetos[k].x+(objetos[k].width/2)))<=30 && abs((detections[j].y+(detections[j].height/2)) - (objetos[k].y+(objetos[k].height/2))) <= 30) {
						resp = true; //Ya esta
					}
					k++;
				}
				if (!resp) {
					objetos.push_back(detections[j]); //nuevo
					ultimoFrame.push_back(i);
					KalmanFilter a;
					lastPrediction = std::chrono::steady_clock::now();
					preparacionKalman(detections[j], &a);
					filtrosK.push_back(a);
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
			KalmanFilter a;

			PrediccionActual = std::chrono::steady_clock::now();

			while (m < numberO) {
				if (i == ultimoFrame[m]) {
					//La i se quita, solo para guardar las imagenes
					//Considerar ultimo frame para ya no calcularlo ej i-5
					kalman(filtrosK[m], objetos[m], imgF,i,true,PrediccionActual,lastPrediction,&a);
					filtrosK[m] = a;
				}
				else {
					if (i - ultimoFrame[m] < 15) {
						kalman(filtrosK[m], objetos[m], imgF, i, false,PrediccionActual,lastPrediction,&a);
						filtrosK[m] = a;
					}
					else
					{
						filtrosK.erase(filtrosK.begin()+m);
						objetos.erase(objetos.begin() + m);
						ultimoFrame.erase(ultimoFrame.begin() + m);
						m--;
						numberO--;
					}
				}
				m++;
			}
			
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
