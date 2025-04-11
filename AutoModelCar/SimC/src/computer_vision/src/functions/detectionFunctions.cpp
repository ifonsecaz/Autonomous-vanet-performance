#include "detectionFunctions.h"

/*
Método que realiza la deteccion sobre la imagen completa
Recibe las máquinas de soporte vectorial y el filtro de cascada, el frame actual y el número del frame
Devuelve la imagen con los rectángulos marcados de las detecciones, el vector de rectángulos y el número
Según se use una sola o las dos máquinas de soporte vectorial, se debe descomentar la función correspondiente y cambiar en result de SVM
*/
void detecVentana(Ptr<SVM>* svm, Ptr<SVM>* svm2,HOGDescriptor* hog,CascadeClassifier* carC, Mat* img, int cont, Mat* imgF, int* numberDetec, vector<Rect>* detections)
//void detecVentana(Ptr<SVM> svm,CascadeClassifier carC, Mat img, int cont, Mat* imgF,int* numberDetec,vector<Rect>* detections)
{
	Mat imgDet;
    img->convertTo(imgDet,CV_8UC3);

    vector< Rect > detectionsCascada;
    vector< Rect > detectionsFinal;
    Mat imgRec;

	//Se realiza la detección con el filtro de cascada sobre la imagen completa, considera un tamaño mínimo de 40x40 pixeles, y máximo de 200x200, va incrementando el tamaño de la ventana por 1.1
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    carC->detectMultiScale(imgDet, detectionsCascada, 1.1, 2, 0, Size(40, 40), Size(200, 200));
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    
    cout << "\n number of detections: " << detectionsCascada.size();
    int aux = 0;
	//Cada detección de cascada se evalúa, primero se toma un área alrededor un 20% más grande
    for (size_t j = 0; j < detectionsCascada.size(); j++)
    {
        int width = detectionsCascada[j].width * 0.2;
        int height = detectionsCascada[j].height * 0.2;
		//Revisa que no rebase la imagen 
        if ((detectionsCascada[j].x - (int)(width/2)) > 0 && (detectionsCascada[j].y - (int)(height/2)) > 0 && (detectionsCascada[j].x + detectionsCascada[j].width + (int)(width/2)) < 640 && (detectionsCascada[j].y + detectionsCascada[j].height + (int)(height/2))<400) {
            imgRec = Mat(detectionsCascada[j].width+width, detectionsCascada[j].height+height, CV_64F, 0.0); //CV_64F 
            imgDet(Rect(detectionsCascada[j].x-(int)(width/2), detectionsCascada[j].y-(int)(height/2), detectionsCascada[j].width+width, detectionsCascada[j].height+height)).copyTo(imgRec(Rect(0, 0, detectionsCascada[j].width+width, detectionsCascada[j].height+height)));
        }
        else {
            imgRec = Mat(detectionsCascada[j].width, detectionsCascada[j].height, CV_64F, 0.0); //CV_64F 
            imgDet(Rect(detectionsCascada[j].x, detectionsCascada[j].y, detectionsCascada[j].width, detectionsCascada[j].height)).copyTo(imgRec(Rect(0, 0, detectionsCascada[j].width, detectionsCascada[j].height)));
        }
        
        float result = predictWithSVM(svm, svm2, hog, &imgRec);
        
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
    imgDet(Rect(510, 260, 130, 130)).copyTo(imgEsquina(Rect(0, 0, 130, 130)));

    float result = predictWithSVM(svm, svm2, hog, &imgEsquina);

    if (result > 0) { //0.0 simulador, caso real 0.3
        aux++;
        detectionsFinal.push_back(Rect(510,260,130,130));
        detectionsFinal.push_back(Rect(510,260,130,130));
    }

    //Esquina inferior izquierda
    imgDet(Rect(0,260, 130, 130)).copyTo(imgEsquina(Rect(0, 0, 130, 130)));

    float result = predictWithSVM(svm, svm2, hog, &imgEsquina);
    
    if (result > 0) { 
        aux++;
        detectionsFinal.push_back(Rect( 0,260, 130, 130));
        detectionsFinal.push_back(Rect(0, 260, 130, 130));
    }
    
	//Se calcula el promedio de los rectángulos detectados que se sobreponen
    groupRectangles(detectionsFinal, 1, 0.60); //groupThreshold numero minimo posible de rectangulos para retenerlo, eps cuanto deben sobreponerse, cuanto más cercano es a 0 no se agrupan
    aux = detectionsFinal.size();
    for (int j = 0; j < detectionsFinal.size(); j++) {
        rectangle(imgDet, detectionsFinal[j], RECTANGLE_COLOR, imgDet.cols / 400 + 1);
        rectangle(imgDet, Rect(detectionsFinal[j].x + (detectionsFinal[j].width / 2)-2, detectionsFinal[j].y + (detectionsFinal[j].height / 2)-2, 4, 4), 250, imgDet.cols / 400 + 1);
    }

	//Se devuelven los valores y se guarda la imagen con las detecciones
    *numberDetec = aux;
    *imgF =  imgDet; ///
    *detections = detectionsFinal;

    saveDetectionImage(&imgDet, cont);

    publishDetectionImage(&imgDet);
}

/*
Método que realiza la prediccion con SVM para mejorar los resultados de detección
Recibe las máquinas de soporte vectorial y la imagen cortada
Devuelve el resultado flotante
*/
float predictWithSVM(Ptr<SVM>* svm, Ptr<SVM>* svm2,HOGDescriptor* hog, Mat* imgRec){
    Mat imgSVM;
    //Para la predicción, la imagen debe ser de 64x64
    resize(*imgRec, imgSVM, Size(64, 64), 0, 0, cv::INTER_AREA);

    //Se aplica un filtro gaussiano, igual se puede probar con motionblur, se da el formato correcto y se extraen sus características
    //GaussianBlur(imgSVM, imgSVMBLUR, Size(5, 5), 0);
    vector< float > descriptorsSVM;

    hog->compute(imgSVM, descriptorsSVM, Size(0, 0), Size(0, 0));
    
    //Se realiza la predicción con SVM
    //Se debe cambiar para usar solo una máquina 
    float result1 = (*svm)->predict(descriptorsSVM);
    float result;
    if (result1 < 0) { //OR
        result = (*svm2)->predict(descriptorsSVM);
    }
    else {
        result = result1;
    }
    
    cout << "\n result " << result;
    return result;
}

/*
Método para realizar detecciones sobre una región, la cual es predicha por kalman
La detección se realiza solo con el clasificador de cascada, o se puede descomentar la función para usar también las máquinas SVM
A partir de la imagen toma una pequeña área más grande, devuelve la imagen, un booleano indicando si lo encontró y la detección
*/
//void detecRegion(Ptr<SVM> svm, Ptr<SVM> svm2, CascadeClassifier carC, Mat img, int cont, int extra, int x, int y, int width, int height, Mat* imgF, bool* encontro, Rect* detections)
void detecRegion(CascadeClassifier* carC, Mat* img, int cont, Rect predRect, Mat* imgF, bool* encontro, Rect* detections)
{

	
	Mat imgDET;
    *encontro= false;
	
	//Recibe la imagen completa y según la predicción recorta una parte, evalua si puede tomar una ventana hasta un 80% más grande
    if (img->channels() == 3) {
        cvtColor(*img, imgDET, COLOR_BGR2GRAY);
    }
    else {
        imgDET = *img;
    }

    if(predRect.x > 0 && predRect.x < 640 && predRect.y > 0 && predRect.y < ROI && predRect.width > 0 && predRect.height > 0){
        
        int x1, y1,width1,height1;
        int extra = (int)0.8 * predRect.width;
        if ((predRect.x + predRect.width + extra)> 640) {
            width1 = 640 - predRect.x + extra;
            x1 = predRect.x - extra;
        }
        else {
            if ((predRect.x-extra)<0) {
                x1 = 0;
                width1 = predRect.width + extra + predRect.x;
            }
            else {
                x1 = predRect.x - extra;
                width1 = predRect.width + 2 * extra;
            }

        }
        if (predRect.y + predRect.height + extra> 480) {
            height1 = 480 - predRect.y + extra;
            y1 = predRect.y - extra;
        }
        else {
            if ((predRect.y - extra) < 0) {
                y1 = 0;
                height1 = predRect.height + extra + predRect.y;
            }
            else {
                y1 = predRect.y - extra;
                height1 = predRect.height + 2 * extra;
            }
        }
        Mat imgROI = Mat(height1, width1, CV_64F, 0.0);

        vector< Rect > detectionsCascada;
        vector< Rect > detectionsFinal;
        
        imgDET(Rect(x1,y1,width1,height1)).copyTo(imgROI);

        //Se realiza la detección de cascada
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        carC->detectMultiScale(imgROI, detectionsCascada, 1.05, 1, 0, Size(width*0.4, height*0.4), Size(width1, height1)); //Dado tamano segun prediccion
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

                float result = predictWithSVM(imgEsquina);
                if (result > 0.3) { //-0.2 o -0.15 ok
                    aux++;
                    //detectionsAux.push_back(detections2[j]);
                    rectangle(img1, Rect(x1 + detections2[j].x, y1 + detections2[j].y, detections2[j].width, detections2[j].height), 200, img1.cols / 400 + 1);
                    //img1(Rect(x1 + detections2[j].x, y1 + detections2[j].y, detections2[j].width, detections2[j].height)).copyTo(imgZ(Rect(x1 + detections2[j].x, y1 + detections2[j].y, detections2[j].width, detections2[j].height)));
                }
                //

            }
            */
            //Se agregan los rectángulos sobrepuestos, se dibujan sobre la imagen y se guarda
            groupRectangles(detectionsCascada, 1, 0.60);
            rectangle(imgDET, Rect(x1 + detectionsCascada[0].x, y1 + detectionsCascada[0].y, detectionsCascada[0].width, detectionsCascada[0].height), 200, imgGRAY.cols / 400 + 1); //Quitar si se incluye hog
            rectangle(imgDET, Rect(x1+detectionsCascada[0].x + detectionsCascada[0].width / 2-2, y1+detectionsCascada[0].y + detectionsCascada[0].height / 2-2, 4, 4), 250, imgGRAY.cols / 400 + 1);

            *detections = Rect(x1 + detectionsCascada[0].x, y1 + detectionsCascada[0].y, detectionsCascada[0].width, detectionsCascada[0].height);
     
            //Se publica la imagen
            publishDetectionImage(&imgDET);
        }
    }
    //Devuelve la imagen original con o sin detecciones
    *imgF = imgDET;
    saveDetectionImage(&imgDET,cont);
}

void saveDetectionImage(Mat* imgDet, int cont){
    String a = "/home/israel/Documents/k/test" + std::to_string(cont) + ".jpg";

    imwrite(a, *imgDet);
}

void publishDetectionImage(Mat* imgDet){
    ros::NodeHandle nh("~");
    //     /detec es el nombre del tópico para las detecciones, publica un mensaje del tipo sensor_msgs: Image
    detec_publisher = nh.advertise<sensor_msgs::Image>("/detec",1);

	//Se convierte la imagen a sensor_msgs y se publica
	sensor_msgs::Image img_msg;
    std_msgs::Header header; 
    //header.seq = counter; 
    header.stamp = ros::Time::now(); 
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC1, *imgDet);
    img_bridge.toImageMsg(img_msg); 
    detec_publisher.publish(img_msg); 
}