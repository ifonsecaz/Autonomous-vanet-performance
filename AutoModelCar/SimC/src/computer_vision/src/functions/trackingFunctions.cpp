#include "trackingFunctions.h"

/*
Filtro de kalman
Realiza la predicción y corrección en caso de que hubiera detecciones, hace la actualización
Guarda una imagen con la predicción
Recibe el filtro, la deteccion, la imagen, un contador, y un booleano para indicar si la detección se encontró en el frame actual 
Devuelve el filtro actualizado
*/
void kalman(KalmanFilter kf, Rect* detections, Mat* img,int i,bool found, std::chrono::steady_clock::time_point* lastP, std::chrono::steady_clock::time_point* Pactual,KalmanFilter* act) {
	auto d = std::chrono::duration_cast<std::chrono::milliseconds>((*Pactual)-(*lastP));
    double dt = std::chrono::duration<double>(d).count();
    //transitionMatrix update
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

        meas.at<float>(0) = detections->x;
        meas.at<float>(1) = detections->y;
        meas.at<float>(2) = detections->width;
        meas.at<float>(3) = detections->height;

        kf.correct(meas);
    }

	//Devuelve el filtro actualizado
    *act = kf;
	//Dibuja la predicción y se guarda la imagen
    if (predRect.x < 640 && predRect.x>0 && predRect.y < 480 && predRect.y>0) {
        cv::rectangle(*img, predRect, CV_RGB(255, 0, 0), 2);
        cv::rectangle(*img, Rect(predRect.x + predRect.width / 2 - 1, predRect.y + predRect.height / 2 - 1, 2, 2), CV_RGB(1, 0, 0), 2);
    }

	savePredictionImage(img,i);

    publishPredictedImage(img);
}

/*
Método para crear el filtro de kalman
Entra la primera vex que se ve un nuevo objeto
Recibe la detección inicial y devuelve el filtro
*/
void preparacionKalman(Rect* detections, KalmanFilter* kalman) {
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
	
    meas.at<float>(0, 0) = detections->x; //Primer punto
    meas.at<float>(1, 0) = detections->y;
    meas.at<float>(2, 0) = detections->width;
    meas.at<float>(3, 0) = detections->height;

	//Se guarda la primera medición como el estado previo y posterior
	//Inicia con una velocidad de cero
    kf.statePre.setTo(0); 
    kf.statePre.at<float>(0, 0) = detections->x;
    kf.statePre.at<float>(1, 0) = detections->y;
    kf.statePre.at<float>(2, 0) = 0;
    kf.statePre.at<float>(3, 0) = 0;
    kf.statePre.at<float>(4, 0) = detections->width;
    kf.statePre.at<float>(5, 0) = detections->height;

    kf.statePost.setTo(0);
    kf.statePost.at<float>(0, 0) = detections->x;
    kf.statePost.at<float>(1, 0) = detections->y;
    kf.statePost.at<float>(2, 0) = 0;
    kf.statePost.at<float>(3, 0) = 0;
    kf.statePost.at<float>(4, 0) = detections->width;
    kf.statePost.at<float>(5, 0) = detections->height;

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

void savePredictionImage(Mat* img, int cont){
    String b = "/home/israel/Documents/k/kalman/res" + std::to_string(cont) + ".jpg";

    imwrite(b, *img);
}

void publishPredictedImage(Mat* img){
	ros::NodeHandle nh("~");
    //     /pred Nombre del tópico con el que se publica la imagen con la predicción
    detec_publisherk = nh.advertise<sensor_msgs::Image>("/pred",1);

    sensor_msgs::Image img_msg;
    std_msgs::Header header; 
    //header.seq = counter; 
    header.stamp = ros::Time::now(); 
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC1, *img);
    img_bridge.toImageMsg(img_msg); 
    detec_publisherk.publish(img_msg);  
}