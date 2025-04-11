#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/ml.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/videoio.hpp"
#include "C:\Users\ifons\source\repos\validacion2\validacion2\HOGImage-master/HOGImage/HOGImage.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/core/cuda.hpp"
#include <iostream>
#include <time.h>
using namespace cv;
using namespace cv::ml;
using namespace std;
//using namespace cuda;


void convert_to_ml(const vector< Mat >& train_samples, Mat& trainData)
{
    //--Convert data
    const int rows = (int)train_samples.size();
    const int cols = (int)std::max(train_samples[0].cols, train_samples[0].rows);
    Mat tmp(1, cols, CV_32FC1); //< used for transposition if needed
    trainData = Mat(rows, cols, CV_32FC1);
    for (size_t i = 0; i < train_samples.size(); ++i)
    {
        CV_Assert(train_samples[i].cols == 1 || train_samples[i].rows == 1);
        if (train_samples[i].cols == 1)
        {
            transpose(train_samples[i], tmp);
            tmp.copyTo(trainData.row((int)i));
        }
        else if (train_samples[i].rows == 1)
        {
            train_samples[i].copyTo(trainData.row((int)i));
        }
    }
}

void computeHOGs(const vector< Mat >& img_lst, vector< Mat >& gradient_lst)
{
    HOGDescriptor hog;
    hog.winSize = Size(64, 64);
    hog.blockSize = Size(16, 16);
    hog.blockStride = Size(4, 4); //Size(0,0)
    hog.cellSize = Size(4, 4);
    hog.nbins = 9;
    hog.derivAperture = 1;
    hog.winSigma = -1;
    //hog.histogramNormType = ;Usa L2Hys por defecto
    hog.L2HysThreshold = 0.2;
    hog.gammaCorrection = 1;
    hog.nlevels = 64;
    hog.signedGradient = false;

    Mat gray;
    vector< float > descriptors;
    std::cout << "\n" << img_lst.size() << "\n";
    for (size_t i = 0; i < img_lst.size(); i++) //recorre todas las imagenes
    {

        hog.compute(img_lst[i], descriptors, Size(0, 0), Size(0, 0)); //winStride y padding
        gradient_lst.push_back(Mat(descriptors).clone());
    }
}



void load_images(const String& dirname, vector< Mat >& img_lst, bool gaussian, bool sobel, int cut)
{
    Mat img2, img3, imgAux;
    vector< String > files;
    glob(dirname, files);
    for (size_t i = 0; i < files.size(); ++i)
    {
        Mat img = imread(files[i]); // load the image
        if (img.empty())
        {
            std::cout << files[i] << " is invalid!" << endl; // invalid image, skip it.
            continue;
        }
        cvtColor(img, img2, COLOR_BGR2GRAY);
        if (gaussian) {
            GaussianBlur(img2, img3, Size(5, 5), 0);
            img2 = img3;
        }
        if (sobel) {
            Mat sobelx, sobely, sobelxy;
            //Sobel(img2, sobelx, CV_64F, 1, 0, 5);
            //Sobel(img2, sobely, CV_64F, 0, 1, 5);
            Sobel(img2, sobelxy, CV_32F, 1, 1, 5);
            img2 = sobelxy; //probar sobely y sobelxy
        }
        if (cut == 0) {
            img2(Rect(6, 6, 52, 52)).copyTo(imgAux);
        }
        else {
            if (cut == 1) {
                img2(Rect(16, 6, 48, 52)).copyTo(imgAux);
            }
            else {
                imgAux = img;
            }
        }

        //img2(Rect(6,6,52,52)).copyTo(imgAux);
        resize(imgAux, img2, Size(64, 64), 0, 0, cv::INTER_AREA);

        //img2.convertTo(img2, CV_8UC3);

        img_lst.push_back(img2);
    }
}

void test_trained_detectorIm(Ptr<SVM> svm, String directorio, vector<int>* res, int* total, int cut)
{
    std::vector<int> cont(2, 0);
    double max = -1;
    double min = 1;
    int limI;
    int limS;
    Mat img0, img1, img2;
    vector< String > files;

    glob(directorio, files);

    *total = files.size();

    for (size_t i = 0; i < files.size(); ++i)
    {
        Mat img = imread(files[i]); // load the image

        HOGDescriptor hog;
        hog.winSize = Size(64, 64);
        hog.blockSize = Size(16, 16);
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


        resize(img, img0, Size(64, 64), 0, 0, cv::INTER_AREA);
        if (cut == 0) {
            img0(Rect(6, 6, 52, 52)).copyTo(img1); 
        }
        else {
            if (cut == 1) {
                img0(Rect(16, 6, 48, 52)).copyTo(img1); //x es row, y col
            }
            else {
                img0 = img1;
            }
        }
        img0 = img1;
        resize(img, img0, Size(64, 64), 0, 0, cv::INTER_AREA);

        cvtColor(img0, img1, COLOR_BGR2GRAY);
        GaussianBlur(img1, img, Size(5, 5), 0);
        img1 = img;
        Mat sobelx, sobely, sobelxy;
        //Sobel(img2, sobelx, CV_64F, 1, 0, 5);
        //Sobel(img2, sobely, CV_64F, 0, 1, 5);
        //Sobel(img0, sobelxy, CV_64F, 1, 1, 5);
        //img2 = sobelxy; //probar sobely y sobelxy
        
        //GaussianBlur(imgClas, imgClas, Size(3, 3), 0);
        vector< float > descriptors;
        img1.convertTo(img2, CV_8UC3);
        hog.compute(img2, descriptors, Size(0, 0), Size(0, 0));

        float result = svm->predict(descriptors);

        if (result > max) {
            max = result;
        }
        if (result < min) {
            min = result;
        }
        //int c = floor((result * 100 + 100) / 5);
        if (result < 0) {
            //c = 0;
            cont[0] = cont[0] + 1;
        }
        else {
            if (result > 0) {
                //c = 39;
                cont[1] = cont[1] + 1;
            }
        }
        //cont[c] = cont[c] + 1;
    }
    *res = cont;
    std::cout << "\nval min: " << min;
    std::cout << "\nval max: " << max;
}

void test_trained_two_detectorIm(Ptr<SVM> svm1,Ptr<SVM> svm2, String directorio, vector<int>* res, int* total) //Falta considerar cortar
{
    std::vector<int> cont(2, 0);
    double max = -1;
    double min = 1;
    int limI;
    int limS;
    Mat img0, img1, img2;
    vector< String > files;

    glob(directorio, files);

    *total = files.size();

    for (size_t i = 0; i < files.size(); ++i)
    {
        Mat img = imread(files[i]); // load the image

        HOGDescriptor hog;
        hog.winSize = Size(64, 64);
        hog.blockSize = Size(16, 16);
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


        resize(img, img0, Size(64, 64), 0, 0, cv::INTER_AREA);
        cvtColor(img0, img1, COLOR_BGR2GRAY);
        GaussianBlur(img1, img, Size(5, 5), 0);
        img1 = img;
        Mat sobelx, sobely, sobelxy;
        //Sobel(img2, sobelx, CV_64F, 1, 0, 5);
        //Sobel(img2, sobely, CV_64F, 0, 1, 5);
        //Sobel(img0, sobelxy, CV_64F, 1, 1, 5);
        //img2 = sobelxy; //probar sobely y sobelxy

        //GaussianBlur(imgClas, imgClas, Size(3, 3), 0);
        vector< float > descriptors;
        img1.convertTo(img2, CV_8UC3);
        hog.compute(img2, descriptors, Size(0, 0), Size(0, 0));

        float result1 = svm1->predict(descriptors);
        float result2 = svm2->predict(descriptors);
        float result = result1 * result2;
        //if (result < 0) {
        //    result = svm2->predict(descriptors);
        //}

        if (result > max) {
            max = result;
        }
        if (result < min) {
            min = result;
        }
        //int c = floor((result * 100 + 100) / 5);
        if (result < 0) {
            //c = 0;
            cont[0] = cont[0] + 1;
        }
        else {
            if (result > 0) {
                //c = 39;
                cont[1] = cont[1] + 1;
            }
        }
        //cont[c] = cont[c] + 1;
    }
    *res = cont;
    std::cout << "\nval min: " << min;
    std::cout << "\nval max: " << max;
}


int main(int argc, char** argv)
{
    String svmFile = "C:/Users/ifons/source/repos/detec2/detec2/my_svmB.yml";
    String imgPos = "C:/Users/ifons/source/repos/validacion2/validacion2/E/Pos";
    String imgPosC = "C:/Users/ifons/source/repos/validacion2/validacion2/E/PosC";
    String imgPosL = "C:/Users/ifons/source/repos/validacion2/validacion2/E/PosL";
    String imgNeg = "C:/Users/ifons/source/repos/validacion2/validacion2/E/Neg";
    String testPos = "C:/Users/ifons/source/repos/validacion/positivas";
    String testNeg = "C:/Users/ifons/source/repos/validacion/negativas";

    //printCudaDeviceInfo(0);

    int m = 1;

    //Test one svm
    if (m == 0) {
        std::cout << "Testing trained detector..." << endl;

        Ptr<SVM> svm;
        svm = StatModel::load<SVM>(svmFile);

        vector<int> res, res2;
        int total1, total2;

        test_trained_detectorIm(svm, testPos, &res, &total1,3);

        std::cout << "\nset validacion Positivos: \n";
        int val = -100;
        int val2 = -95;
        for (int i = 0; i < res.size(); i++) {
            std::cout << "___" << val << " - " << val2 << ": " << res[i];
            val = val2;
            val2 = val2 + 5;
        }

        test_trained_detectorIm(svm, testNeg, &res2, &total2,3);

        std::cout << "\nset validacion Negativos: \n";
        val = -100;
        val2 = -95;
        for (int i = 0; i < res2.size(); i++) {
            std::cout << " ___" << val << " - " << val2 << ": " << res2[i];
            val = val2;
            val2 = val2 + 5;
        }

        std::cout << "\nRes: \n";
        double fp, tp, fn, tn;
        int ac = 0;
        int ac2 = 0;
        //val = -100;
        //val2 = -95;
        //for (int i = 0; i < res.size(); i++) {
        //    std::cout << "\n" << val << " - " << val2 << ": ";
            //ac = ac + res[i];//Positivos
            //ac2 = ac2 + res2[i];//Negativos
        tn = double(res2[0]) / double(total2);
        fp = (double(res2[1])) / double(total2);
        fn = double(res[0]) / double(total1);
        tp = double((res[1])) / double(total1);
        std::cout << "\nTN: " << tn << " FP: " << fp << " FN: " << fn << " TP: " << tp;
        val = val2;
        val2 = val2 + 5;
        //}

        test_trained_detectorIm(svm, imgPos, &res, &total1,3);

        std::cout << "\nset Positivos: \n";
        val = -100;
        val2 = -95;
        for (int i = 0; i < res.size(); i++) {
            std::cout << "___" << val << " - " << val2 << ": " << res[i];
            val = val2;
            val2 = val2 + 5;
        }

        test_trained_detectorIm(svm, imgNeg, &res2, &total2,3);

        std::cout << "\nset Negativos: \n";
        val = -100;
        val2 = -95;
        for (int i = 0; i < res2.size(); i++) {
            std::cout << "___" << val << " - " << val2 << ": " << res2[i];
            val = val2;
            val2 = val2 + 5;
        }

        std::cout << "\nRes: \n";
        fp, tp, fn, tn;
        ac = 0;
        ac2 = 0;
        val = -100;
        val2 = -95;
        for (int i = 0; i < res.size(); i++) {
            std::cout << "\n" << val << " - " << val2 << ": ";
            ac = ac + res[i];//Positivos
            ac2 = ac2 + res2[i];//Negativos
            tn = double(ac2) / double(total2);
            fp = (double(total2 - ac2)) / double(total2);
            fn = double(ac) / double(total1);
            tp = double((total1 - ac)) / double(total1);
            std::cout << "\nTN: " << tn << " FP: " << fp << " FN: " << fn << " TP: " << tp;
            val = val2;
            val2 = val2 + 5;
        }


    }
    else {
        //Train and test svm
        if (m == 2) {
            bool train_twice = false;
            bool gaussian = true;
            bool sobel = false;

            vector< Mat > pos_lst, neg_lst;
            vector<Mat> gradient_lst;
            vector< int > labels;
            clog << "Positive images are being loaded...";
            load_images(imgPos, pos_lst, gaussian, sobel,3);
            clog << "Negative images are being loaded...";
            load_images(imgNeg, neg_lst, gaussian, sobel,3);

            //for (int i = 0; i < pos_lst.size(); i++) {
            //    clog << "\n " <<pos_lst[i].ptr();
            //}

            computeHOGs(pos_lst, gradient_lst);

            size_t positive_count = gradient_lst.size();
            labels.assign(positive_count, +1);

            computeHOGs(neg_lst, gradient_lst);
            size_t negative_count = gradient_lst.size() - positive_count;
            labels.insert(labels.end(), negative_count, -1);
            clog << "\nInicio\n";

            Mat train_data;
            convert_to_ml(gradient_lst, train_data);

            //cout << "M = " << endl << " " << train_data << endl << endl;

            //for (int i = 0; i < labels.size(); i++) {
            //    clog << "\n" << labels[i];
            //}

            Ptr< SVM > svm = SVM::create();

            //double> C;
            //C.insert(C.end(), { 0.001,0.01, 0.1, 1, 10, 100, 1000,10000,100000,1000000,10000000,100000000,1000000000,10000000000,100000000000,1000000000000,10000000000000,100000000000000,1000000000000000,10000000000000000 });
            //vector<double> p;
            //p.insert(p.end(), { 0.001, 0.01,0.1,0.5,0.8});

            //vector<double> gamma;
            //gamma.insert(gamma.end(), { 0.000002, 0.00002, 0.0002, 0.002,0.02,0.2,2,20,200,2000 });


            //if l is 0 for linear, 1-5 rbf with correspondant gamma values

            svm->setType(SVM::C_SVC);
            svm->setKernel(SVM::RBF);

            svm->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER, (int)1e8, 1e-6));

            //cv::Mat1f weights(1, 2);
            //weights(0, 0) = 2;
            //weights(0, 1) = 8;
            //svm->setClassWeights(weights);

            ParamGrid CvParamGrid_C(pow(2.0, -2), pow(2.0, 5), pow(2.0, 1));
            ParamGrid CvParamGrid_gamma(pow(2.0, -5), pow(2.0, 3), pow(2.0, 1));
            ParamGrid CvParamGrid_p(0.1, 1.0, 0);
            ParamGrid CvParamGrid_nu(pow(2.0, -2), pow(2.0, 5), 0);
            ParamGrid CvParamGrid_coeff(pow(2.0, -2), pow(2.0, 5), 0);
            ParamGrid CvParamGrid_deg(pow(2.0, -2), pow(2.0, 5), 0);

            //C=.01
            //gamma = 0.0015 o 0.0025

            //  svm->setCoef0(1.0);
            // svm->setDegree(3);
            //svm->setNu(0.5);
            //svm->setP(0.1); // for EPSILON_SVR, epsilon in loss function?

            //ParamGrid CvParamGrid_C(0.01, 10000000000000000, 10);
            //ParamGrid CvParamGrid_Gamma(0.001, 1000000, 10);

            //svm->setDegree(.65);
            //svm->train(ml::TrainData::create(train_data, ml::ROW_SAMPLE, labels));

            //svm->trainAuto(ml::TrainData::create(train_data, ml::ROW_SAMPLE, labels));
            //svm->getDefaultGrid(SVM::C)
            svm->trainAuto(ml::TrainData::create(train_data, ml::ROW_SAMPLE, labels), 10, svm->getDefaultGrid(SVM::C), svm->getDefaultGrid(SVM::GAMMA), CvParamGrid_p, CvParamGrid_nu, CvParamGrid_coeff, CvParamGrid_deg, false);
            //Cambiar tamanos de la imagen y recortes, volver a probar sobel
            Mat SV = svm->getSupportVectors();
            Mat USV = svm->getUncompressedSupportVectors();

            std::cout << "\nSupport Vectors: " << SV.rows << endl;
            std::cout << "\nUncompressed Support Vectors: " << USV.rows << endl;


            std::cout << "\nParameters C: " << svm->getC() << " gamma: " << svm->getGamma() << " Type: " << svm->getType() << " Kernel: " << svm->getKernelType() << "Term: " << svm->getTermCriteria().epsilon << " maxCount" << svm->getTermCriteria().maxCount << " pesos: " << svm->getClassWeights() << " count" << svm->getTermCriteria().COUNT << " eps: " << svm->getTermCriteria().EPS;

            vector<int> res, res2;
            int total1, total2;

            test_trained_detectorIm(svm, testPos, &res, &total1,3);

            std::cout << "\nset validacion Positivos: \n";
            int val = -100;
            int val2 = -95;
            for (int i = 0; i < res.size(); i++) {
                //std::cout << "___" << val << " - " << val2 << ": " << res[i];
                val = val2;
                val2 = val2 + 5;
            }

            test_trained_detectorIm(svm, testNeg, &res2, &total2,3);

            //std::cout << "\nset validacion Negativos: \n";
            val = -100;
            val2 = -95;
            for (int i = 0; i < res2.size(); i++) {
                //std::cout << " ___" << val << " - " << val2 << ": " << res2[i];
                val = val2;
                val2 = val2 + 5;
            }

            std::cout << "\nRes Validacion: \n";
            double fp, tp, fn, tn;
            int ac = 0;
            int ac2 = 0;
            //val = -100;
            //val2 = -95;
            //for (int i = 0; i < res.size(); i++) {
                //std::cout << "\n" << val << " - " << val2 << ": ";
                //ac = ac + res[i];//Positivos
                //ac2 = ac2 + res2[i];//Negativos
            tn = double(res2[0]) / double(total2);
            fp = (double(res2[1])) / double(total2);
            fn = double(res[0]) / double(total1);
            tp = double((res[1])) / double(total1);
            std::cout << "\nTN: " << tn << " FP: " << fp << " FN: " << fn << " TP: " << tp;
            val = val2;
            val2 = val2 + 5;
            //}

            test_trained_detectorIm(svm, imgPos, &res, &total1,3);

            //std::cout << "\nset Positivos: \n";
            val = -100;
            val2 = -95;
            for (int i = 0; i < res.size(); i++) {
                //std::cout << "___" << val << " - " << val2 << ": " << res[i];
                val = val2;
                val2 = val2 + 5;
            }

            test_trained_detectorIm(svm, imgNeg, &res2, &total2,3);

            //std::cout << "\nset Negativos: \n";
            val = -100;
            val2 = -95;
            for (int i = 0; i < res2.size(); i++) {
                //std::cout << "___" << val << " - " << val2 << ": " << res2[i];
                val = val2;
                val2 = val2 + 5;
            }

            std::cout << "\nRes Entrenamiento: \n";
            fp, tp, fn, tn;
            ac = 0;
            ac2 = 0;
            val = -100;
            val2 = -95;
            for (int i = 0; i < res.size(); i++) {
                std::cout << "\n" << val << " - " << val2 << ": ";
                ac = ac + res[i];//Positivos
                ac2 = ac2 + res2[i];//Negativos
                tn = double(ac2) / double(total2);
                fp = (double(total2 - ac2)) / double(total2);
                fn = double(ac) / double(total1);
                tp = double((total1 - ac)) / double(total1);
                std::cout << "\nTN: " << tn << " FP: " << fp << " FN: " << fn << " TP: " << tp;
                val = val2;
                val2 = val2 + 5;
            }


            svm->save("my_svmML2.yml");


        }
        else {
            //train both svm, final
            if (m == 1) {
                bool train_twice = false;
                bool gaussian = true;
                bool sobel = false;

                vector< Mat > pos_lstC,pos_lstL, neg_lst;
                vector<Mat> gradient_lstC,gradient_lstL;
                vector< int > labelsC,labelsL;
                clog << "Positive images center are being loaded...";
                load_images(imgPosC, pos_lstC, gaussian, sobel,3);
                clog << "Positive images side are being loaded...";
                load_images(imgPosL, pos_lstL, gaussian, sobel,3);
                clog << "Negative images are being loaded...";
                load_images(imgNeg, neg_lst, gaussian, sobel,3);

                //for (int i = 0; i < pos_lst.size(); i++) {
                //    clog << "\n " <<pos_lst[i].ptr();
                //}

                computeHOGs(pos_lstC, gradient_lstC);

                computeHOGs(pos_lstL, gradient_lstL);


                size_t positive_countC = gradient_lstC.size();
                labelsC.assign(positive_countC, +1);

                size_t positive_countL = gradient_lstL.size();
                labelsL.assign(positive_countL, +1);

                computeHOGs(neg_lst, gradient_lstC);
                computeHOGs(neg_lst, gradient_lstL);

                size_t negative_countC = gradient_lstC.size() - positive_countC;
                labelsC.insert(labelsC.end(), negative_countC, -1);

                size_t negative_countL = gradient_lstL.size() - positive_countL;
                labelsL.insert(labelsL.end(), negative_countL, -1);

                clog << "\nInicio\n";

                Mat train_dataC,train_dataL;
                convert_to_ml(gradient_lstC, train_dataC);
                
                convert_to_ml(gradient_lstL, train_dataL);

                //cout << "M = " << endl << " " << train_data << endl << endl;

                //for (int i = 0; i < labels.size(); i++) {
                //    clog << "\n" << labels[i];
                //}

                Ptr< SVM > svmC = SVM::create();
                Ptr< SVM > svmL = SVM::create();


                //double> C;
                //C.insert(C.end(), { 0.001,0.01, 0.1, 1, 10, 100, 1000,10000,100000,1000000,10000000,100000000,1000000000,10000000000,100000000000,1000000000000,10000000000000,100000000000000,1000000000000000,10000000000000000 });
                //vector<double> p;
                //p.insert(p.end(), { 0.001, 0.01,0.1,0.5,0.8});

                //vector<double> gamma;
                //gamma.insert(gamma.end(), { 0.000002, 0.00002, 0.0002, 0.002,0.02,0.2,2,20,200,2000 });


                //if l is 0 for linear, 1-5 rbf with correspondant gamma values

                svmC->setType(SVM::C_SVC);
                svmC->setKernel(SVM::RBF);

                svmC->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER, (int)1e8, 1e-6));

                svmL->setType(SVM::C_SVC);
                svmL->setKernel(SVM::RBF);

                svmL->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER, (int)1e8, 1e-6));

                cv::Mat1f weights(1, 2);
                weights(0, 0) = 0.2;
                weights(0, 1) = 0.8;
                /*
                ParamGrid CvParamGrid_C(pow(2.0, -2), pow(2.0, 5), pow(2.0, 1));
                ParamGrid CvParamGrid_gamma(pow(2.0, -5), pow(2.0, 3), pow(2.0, 1));
                ParamGrid CvParamGrid_p(0.1, 1.0, 0);
                ParamGrid CvParamGrid_nu(pow(2.0, -2), pow(2.0, 5), 0);
                ParamGrid CvParamGrid_coeff(pow(2.0, -2), pow(2.0, 5), 0);
                ParamGrid CvParamGrid_deg(pow(2.0, -2), pow(2.0, 5), 0);
                */

                //C=.01
                //gamma = 0.0015 o 0.0025

                svmC->setC(62.5);
                svmL->setC(12.5);
                svmC->setGamma(0.00225);
                svmL->setGamma(0.00225);
                svmC->setClassWeights(weights);
                svmL->setClassWeights(weights);

                // svm->setDegree(3);
                //svm->setNu(0.5);
                //svm->setP(0.1); // for EPSILON_SVR, epsilon in loss function?

                //ParamGrid CvParamGrid_C(0.01, 10000000000000000, 10);
                //ParamGrid CvParamGrid_Gamma(0.001, 1000000, 10);

                //svm->setDegree(.65);
                //svmC->train(ml::TrainData::create(train_dataC, ml::ROW_SAMPLE, labelsC));

                //svm->trainAuto(ml::TrainData::create(train_data, ml::ROW_SAMPLE, labels));
                //svm->getDefaultGrid(SVM::C)
                clog << " maquina1\n";
                //svmC->trainAuto(ml::TrainData::create(train_dataC, ml::ROW_SAMPLE, labelsC), 10, svmC->getDefaultGrid(SVM::C), svmC->getDefaultGrid(SVM::GAMMA), CvParamGrid_p, CvParamGrid_nu, CvParamGrid_coeff, CvParamGrid_deg, false);
                svmC->train(ml::TrainData::create(train_dataC, ml::ROW_SAMPLE, labelsC));

                clog << " maquina2 \n";
                //svmL->trainAuto(ml::TrainData::create(train_dataL, ml::ROW_SAMPLE, labelsL), 10, svmL->getDefaultGrid(SVM::C), svmL->getDefaultGrid(SVM::GAMMA), CvParamGrid_p, CvParamGrid_nu, CvParamGrid_coeff, CvParamGrid_deg, false);
                svmL->train(ml::TrainData::create(train_dataL, ml::ROW_SAMPLE, labelsL));

                //Cambiar tamanos de la imagen y recortes, volver a probar sobel
                Mat SV = svmC->getSupportVectors();
                Mat USV = svmC->getUncompressedSupportVectors();

                std::cout << "\nSupport Vectors: " << SV.rows << endl;
                std::cout << "\nUncompressed Support Vectors: " << USV.rows << endl;

                SV = svmL->getSupportVectors();
                USV = svmL->getUncompressedSupportVectors();

                std::cout << "\nSupport Vectors: " << SV.rows << endl;
                std::cout << "\nUncompressed Support Vectors: " << USV.rows << endl;


                std::cout << "\nMaquina 1: Parameters C: " << svmC->getC() << " gamma: " << svmC->getGamma() << " Type: " << svmC->getType() << " Kernel: " << svmC->getKernelType() << "Term: " << svmC->getTermCriteria().epsilon << " maxCount" << svmC->getTermCriteria().maxCount << " pesos: " << svmC->getClassWeights() << " count" << svmC->getTermCriteria().COUNT << " eps: " << svmC->getTermCriteria().EPS;

                std::cout << "\nMaquina 2: Parameters C: " << svmL->getC() << " gamma: " << svmL->getGamma() << " Type: " << svmL->getType() << " Kernel: " << svmL->getKernelType() << "Term: " << svmL->getTermCriteria().epsilon << " maxCount" << svmL->getTermCriteria().maxCount << " pesos: " << svmL->getClassWeights() << " count" << svmL->getTermCriteria().COUNT << " eps: " << svmL->getTermCriteria().EPS;

                svmC->save("my_svmML4C.yml");
                svmL->save("my_svmML4L.yml");

                vector<int> res, res2;
                int total1, total2;

                test_trained_two_detectorIm(svmC,svmL, testPos, &res, &total1);
                std::cout << "\nset validacion Positivos: \n";
                int val = -100;
                int val2 = -95;
                for (int i = 0; i < res.size(); i++) {
                    //std::cout << "___" << val << " - " << val2 << ": " << res[i];
                    val = val2;
                    val2 = val2 + 5;
                }

                test_trained_two_detectorIm(svmC, svmL,testNeg, &res2, &total2);

                //std::cout << "\nset validacion Negativos: \n";
                val = -100;
                val2 = -95;
                for (int i = 0; i < res2.size(); i++) {
                    //std::cout << " ___" << val << " - " << val2 << ": " << res2[i];
                    val = val2;
                    val2 = val2 + 5;
                }

                std::cout << "\nRes Validacion: \n";
                double fp, tp, fn, tn;
                int ac = 0;
                int ac2 = 0;
                //val = -100;
                //val2 = -95;
                //for (int i = 0; i < res.size(); i++) {
                    //std::cout << "\n" << val << " - " << val2 << ": ";
                    //ac = ac + res[i];//Positivos
                    //ac2 = ac2 + res2[i];//Negativos
                tn = double(res2[0]) / double(total2);
                fp = (double(res2[1])) / double(total2);
                fn = double(res[0]) / double(total1);
                tp = double((res[1])) / double(total1);
                std::cout << "\nTN: " << tn << " FP: " << fp << " FN: " << fn << " TP: " << tp;
                val = val2;
                val2 = val2 + 5;
                //}


                test_trained_detectorIm(svmC, testPos, &res, &total1,3);

                std::cout << "\nset validacion Positivos: \n";
                val = -100;
                val2 = -95;
                for (int i = 0; i < res.size(); i++) {
                    //std::cout << "___" << val << " - " << val2 << ": " << res[i];
                    val = val2;
                    val2 = val2 + 5;
                }

                test_trained_detectorIm(svmC, testNeg, &res2, &total2,3);

                //std::cout << "\nset validacion Negativos: \n";
                val = -100;
                val2 = -95;
                for (int i = 0; i < res2.size(); i++) {
                    //std::cout << " ___" << val << " - " << val2 << ": " << res2[i];
                    val = val2;
                    val2 = val2 + 5;
                }

                std::cout << "\nRes Validacion: \n";
                //double fp, tp, fn, tn;
                ac = 0;
                ac2 = 0;
                //val = -100;
                //val2 = -95;
                //for (int i = 0; i < res.size(); i++) {
                    //std::cout << "\n" << val << " - " << val2 << ": ";
                    //ac = ac + res[i];//Positivos
                    //ac2 = ac2 + res2[i];//Negativos
                tn = double(res2[0]) / double(total2);
                fp = (double(res2[1])) / double(total2);
                fn = double(res[0]) / double(total1);
                tp = double((res[1])) / double(total1);
                std::cout << "\nTN: " << tn << " FP: " << fp << " FN: " << fn << " TP: " << tp;
                val = val2;
                val2 = val2 + 5;
                //}

                test_trained_detectorIm(svmL, testPos, &res, &total1,3);

                std::cout << "\nset validacion Positivos: \n";
                val = -100;
                val2 = -95;
                for (int i = 0; i < res.size(); i++) {
                    //std::cout << "___" << val << " - " << val2 << ": " << res[i];
                    val = val2;
                    val2 = val2 + 5;
                }

                test_trained_detectorIm(svmL, testNeg, &res2, &total2,3);

                //std::cout << "\nset validacion Negativos: \n";
                val = -100;
                val2 = -95;
                for (int i = 0; i < res2.size(); i++) {
                    //std::cout << " ___" << val << " - " << val2 << ": " << res2[i];
                    val = val2;
                    val2 = val2 + 5;
                }

                std::cout << "\nRes Validacion: \n";
                fp, tp, fn, tn;
                ac = 0;
                ac2 = 0;
                //val = -100;
                //val2 = -95;
                //for (int i = 0; i < res.size(); i++) {
                    //std::cout << "\n" << val << " - " << val2 << ": ";
                    //ac = ac + res[i];//Positivos
                    //ac2 = ac2 + res2[i];//Negativos
                tn = double(res2[0]) / double(total2);
                fp = (double(res2[1])) / double(total2);
                fn = double(res[0]) / double(total1);
                tp = double((res[1])) / double(total1);
                std::cout << "\nTN: " << tn << " FP: " << fp << " FN: " << fn << " TP: " << tp;
                val = val2;
                val2 = val2 + 5;
                //}

                test_trained_detectorIm(svmC, imgPos, &res, &total1,3);

                //std::cout << "\nset Positivos: \n";
                val = -100;
                val2 = -95;
                for (int i = 0; i < res.size(); i++) {
                    //std::cout << "___" << val << " - " << val2 << ": " << res[i];
                    val = val2;
                    val2 = val2 + 5;
                }

                test_trained_detectorIm(svmC, imgNeg, &res2, &total2,3);

                //std::cout << "\nset Negativos: \n";
                val = -100;
                val2 = -95;
                for (int i = 0; i < res2.size(); i++) {
                    //std::cout << "___" << val << " - " << val2 << ": " << res2[i];
                    val = val2;
                    val2 = val2 + 5;
                }

                std::cout << "\nRes Entrenamiento: \n";
                fp, tp, fn, tn;
                ac = 0;
                ac2 = 0;
                val = -100;
                val2 = -95;
                for (int i = 0; i < res.size(); i++) {
                    std::cout << "\n" << val << " - " << val2 << ": ";
                    ac = ac + res[i];//Positivos
                    ac2 = ac2 + res2[i];//Negativos
                    tn = double(ac2) / double(total2);
                    fp = (double(total2 - ac2)) / double(total2);
                    fn = double(ac) / double(total1);
                    tp = double((total1 - ac)) / double(total1);
                    std::cout << "\nTN: " << tn << " FP: " << fp << " FN: " << fn << " TP: " << tp;
                    val = val2;
                    val2 = val2 + 5;
                }

                test_trained_detectorIm(svmL, imgPos, &res, &total1,3);

                //std::cout << "\nset Positivos: \n";
                val = -100;
                val2 = -95;
                for (int i = 0; i < res.size(); i++) {
                    //std::cout << "___" << val << " - " << val2 << ": " << res[i];
                    val = val2;
                    val2 = val2 + 5;
                }

                test_trained_detectorIm(svmL, imgNeg, &res2, &total2,3);

                //std::cout << "\nset Negativos: \n";
                val = -100;
                val2 = -95;
                for (int i = 0; i < res2.size(); i++) {
                    //std::cout << "___" << val << " - " << val2 << ": " << res2[i];
                    val = val2;
                    val2 = val2 + 5;
                }

                std::cout << "\nRes Entrenamiento: \n";
                fp, tp, fn, tn;
                ac = 0;
                ac2 = 0;
                val = -100;
                val2 = -95;
                for (int i = 0; i < res.size(); i++) {
                    std::cout << "\n" << val << " - " << val2 << ": ";
                    ac = ac + res[i];//Positivos
                    ac2 = ac2 + res2[i];//Negativos
                    tn = double(ac2) / double(total2);
                    fp = (double(total2 - ac2)) / double(total2);
                    fn = double(ac) / double(total1);
                    tp = double((total1 - ac)) / double(total1);
                    std::cout << "\nTN: " << tn << " FP: " << fp << " FN: " << fn << " TP: " << tp;
                    val = val2;
                    val2 = val2 + 5;
                }



            }
        }
    }
    //visualizar resultados, HOGImage
    if (m == 3) {
        //Mat image = imread("C:/Users/ifons/source/repos/kalman/kalman/validacion/negativas/img365.png");
        //Mat image = imread("C:/Users/ifons/source/repos/validacion2/validacion2/E/PosL/255.png");
        Mat image = imread("C:/Users/ifons/source/repos/validacion2/validacion2/E/PosC/68.png");

       // C:\Users\ifons\source\repos\validacion2\validacion2\E\PosL
        //Mat image = imread("C:/Users/ifons/source/repos/kalman/kalman/validacion/positivas/img6.png");
        //HOGDescriptor hogDesc(image.size(),
        //    Size(20, 20),
        //    Size(10, 10),
        //    Size(10, 10),
        //    9);
        HOGDescriptor hog;
        hog.winSize = Size(64, 64);
        hog.blockSize = Size(16, 16); //(16,16)
        hog.cellSize = Size(4, 4);
        hog.blockStride = Size(4, 4);
        hog.nbins = 9; //9 a 12
        hog.derivAperture = 1;
        hog.winSigma = 4;
        //hog.histogramNormType = 0;
        hog.L2HysThreshold = 2.0000000000000001e-01;
        hog.gammaCorrection = 1;
        hog.nlevels = 64;
        hog.signedGradient = 0;
        vector< float > descriptors;
        
        Mat img0, img1, imgF,img2,img3;
        GaussianBlur(image, img0, Size(5, 5), 0);


        //img0(Rect(6, 6, 52, 52)).copyTo(img1);
        
        //img0(Rect(16, 6, 48, 52)).copyTo(img1);
        //resize(img1, img3, Size(64, 64), 0, 0, cv::INTER_AREA);
        //cvtColor(img2, imgF, COLOR_BGR2GRAY);
        imgF = img0;
        hog.compute(imgF, descriptors, Size(0, 0), Size(0, 0));


        vector<float> desc;
        string name = "HOG window";
        namedWindow(name);
        imshow(name, HOGImage(imgF, hog, 5, 3));
        waitKey();
    }
}