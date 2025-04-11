#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
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
#include <cv_bridge/cv_bridge.h> 
#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/ml/ml.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/opencv.hpp"

#include "prueba/detec.h"
#include "prueba/detecTiempos.h"
#include "prueba/detecArray.h"
//ros::NodeHandle nh;

//ros::Subscriber sub
using namespace cv;
using namespace cv::ml;
using namespace std;


ros::Publisher velocity_publisher; 
ros::Publisher steering_publisher; 

int carril=2; //3 c
int ncarril=2;
int termino=0;
int evadir;

sensor_msgs::PointCloud2 msg2;
int16_t esquiva;
std::vector<vector<Rect>> deteccionesA; //Las detecciones ya estan actualizadas
int numDet;
std::vector<int> distancias;

std::vector<float> xA; // o double
std::vector<float> yA;
std::vector<float> zA;

void esquivaCallback()
{	
  //esquiva=msg.data;
  //ROS_INFO_STREAM("Esquivar: ");
  //suscribe a detecciones dan el centro
  //3 carriles->2 rectas
  //ec1 m -1.17, b 619
  double m=1.62;
  double b=-137.5;
  double m2=-1.65;
  double b2=793;
  int x0;
  int y0;
  int yC;
  int yC2;
  //y0 x0
  esquiva=0;
  ROS_INFO_STREAM("REC numDet"<<numDet);
  
  for(int i=0;i<numDet;i++){
  	for(int j=0;j<9;j++){
  		x0=deteccionesA[i][j].x+(int)deteccionesA[i][j].width/2;
  		y0=400-(deteccionesA[i][j].y+(int)deteccionesA[i][j].height/2);
  		yC=m*x0+b;
  		yC2=m2*x0+b2;
  		std::cout << "y: "<<y0<<" x: "<<x0<<" yC "<<yC<<" yC2 "<<yC2;
  		if(y0<157){//155
	  		if(y0<yC and y0<yC2){
  				esquiva=1;
  				termino=1;
  				ROS_INFO_STREAM("Esquivar ");
  			}//mismo carril	
  		}
  	}
  }
//  yC=m*x0+b
  //ec2 m 1.16, b -405.28
  //yC2=m*x0+b
  //if(y0>yC and y0<yC2)
  //obs en carril 2
  //if(y0<yC)
  //carril 1
  //if(y0>yC2)
  //carril 3
  /*if(termino==0){
  	evadir=0;
  	termino=1;
  }*/
}

void camaraRGBCallback(const sensor_msgs::PointCloud2& msg)
{
  msg2=msg;
  
}

void deteccionesCallback(const prueba::detecArray& msg)
{
  deteccionesA.clear();
  numDet=(int)msg.numDetec;
  vector<Rect> deteccionesInd;
  deteccionesInd.clear();
  for(int i=0; i<msg.numDetec; i++){
  	deteccionesInd.push_back(Rect(msg.array[i].posA.x,msg.array[i].posA.y,msg.array[i].posA.width,msg.array[i].posA.height));
  	for(int j=0;j<10;j++){
  	  	deteccionesInd.push_back(Rect(msg.array[i].posSig[j].x,msg.array[i].posSig[j].y,msg.array[i].posSig[j].width,msg.array[i].posSig[j].height));
  	}
  	deteccionesA.push_back(deteccionesInd);
  }
  esquivaCallback();

}

void distancia()
{

  distancias.clear();
//lee msg2 con la camara y la deteccion actual
	//extraigo distancia de la camara
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(msg2,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pt_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::fromPCLPointCloud2(pcl_pc2,*pt_cloud);
	
  xA.clear();
  yA.clear();
  zA.clear();

  for(int i = 0; i <  640*480; ++i){
		xA.push_back(pt_cloud->points[i].x);
		yA.push_back(pt_cloud->points[i].y);
		zA.push_back(pt_cloud->points[i].z);
  }
  
  //Convierto a matriz valores de z, x y
  Mat Mz=Mat(480,640,CV_64F,0.0);
  memcpy(Mz.data,zA.data(),zA.size()*sizeof(float));
  
  Mat Mx=Mat(480,640,CV_64F,0.0);
  memcpy(Mx.data,xA.data(),xA.size()*sizeof(float));
  
  Mat My=Mat(480,640,CV_64F,0.0);
  memcpy(My.data,yA.data(),yA.size()*sizeof(float));
  

  
  for (int i=0;i<numDet;i++){
 	double zProm;
 	double yProm;
 	double xProm;
 	double a;
  	int cont=0;
  	
  	Scalar m, stdv;
  	
  	Mat imgAux=Mat(deteccionesA[numDet][0].width, deteccionesA[numDet][0].height, CV_64F, 0.0); //CV_64F
  
        Mz(Rect(deteccionesA[numDet][0].x, deteccionesA[numDet][0].y, deteccionesA[numDet][0].width, deteccionesA[numDet][0].height)).copyTo(imgAux(Rect(0, 0, deteccionesA[numDet][0].width, deteccionesA[numDet][0].height)));
        
	//cv::meanStdDev(src, mean, stddev);
	meanStdDev(imgAux, m, stdv);
	
	zProm=sum(m)[0]; //la imagen es de un solo canal
	
	Mx(Rect(deteccionesA[numDet][0].x, deteccionesA[numDet][0].y, deteccionesA[numDet][0].width, deteccionesA[numDet][0].height)).copyTo(imgAux(Rect(0, 0, deteccionesA[numDet][0].width, deteccionesA[numDet][0].height)));
        
	//cv::meanStdDev(src, mean, stddev);
	meanStdDev(imgAux, m, stdv);
	xProm=sum(m)[0];
	
	My(Rect(deteccionesA[numDet][0].x, deteccionesA[numDet][0].y, deteccionesA[numDet][0].width, deteccionesA[numDet][0].height)).copyTo(imgAux(Rect(0, 0, deteccionesA[numDet][0].width, deteccionesA[numDet][0].height)));
        
	//cv::meanStdDev(src, mean, stddev);
	meanStdDev(imgAux, m, stdv);
	yProm=sum(m)[0];
	
	distancias.push_back(sqrt(pow(2.0,zProm)+pow(2.0,yProm)+pow(2.0,xProm)));
  }
}



void movimiento(int cont){
	ros::NodeHandle nh ("~");
	velocity_publisher = nh.advertise<std_msgs::Int16>("/a0/manual_control/speed", 1);//AutoNOMOS_mini/manual_control/speed
        steering_publisher = nh.advertise<std_msgs::Int16>("/a0/manual_control/steering", 1);
	std_msgs::Int16 velocity_message;
  	std_msgs::Int16 steering_message;
	//if(cont<220){
		if((esquiva==1 or termino!=0)&& evadir<70){
		    	ROS_INFO_STREAM("CambiandoCarril");
			if(carril==1){
				if(evadir<35){
					velocity_message.data=-80;
					steering_message.data=20;
					ncarril=2;	
					
				}
				else{
					velocity_message.data=-80;
					steering_message.data=160;
				}
			}
			else{
				if(carril==3){
					if(evadir<35){
						velocity_message.data=-80;
						steering_message.data=160;	
						ncarril=2;
						
					}
					else{
						velocity_message.data=-80;
						steering_message.data=20;
					}
				}
				else{
					if(evadir<35){
						velocity_message.data=-80;
						steering_message.data=160;	
						ncarril=1;
						
					}
					else{
						velocity_message.data=-80;
						steering_message.data=20;
					}
				}
			}
			    	evadir=evadir+1;
			    	velocity_publisher.publish(velocity_message);
    				steering_publisher.publish(steering_message);
		}
		else{

    		if(evadir==68){
    		    		steering_message.data=90;
    		    		velocity_message.data=-100;
    		    		velocity_publisher.publish(velocity_message);
    				steering_publisher.publish(steering_message);
    		}

    		termino=0;
    		    	evadir=0;
    		esquiva=0;
    		carril=ncarril;
    		}
	/*}
	else{
		if(carril==1){
			velocity_message.data=-80;
    			steering_message.data=20;
    		}
    		else{
    			if(carril==2){
				velocity_message.data=-70;
    				steering_message.data=10;
    			}
    			else{
    				velocity_message.data=-60;
    				steering_message.data=0;
    			}
    		}
    		
	}*/
	

    	ROS_INFO_STREAM("Velocidad: " << velocity_message << " Steering: " << steering_message);
}


int main(int argc, char** argv){
  ros::init(argc, argv, "mov_carro");

  ros::NodeHandle nh("~");
  ros::Subscriber camara_sub = nh.subscribe("/app/camera/points", 1, camaraRGBCallback);///depth/points
  //ros::Subscriber esquiva = nh.subscribe("/obs", 10, esquivaCallback);
  ros::Subscriber detecciones = nh.subscribe("/detecciones", 1, deteccionesCallback);

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  
  //ros::Publisher velocity_publisher = nh.advertise<std_msgs::Int16>("/AutoNOMOS_mini/manual_control/speed", 1);
  //ros::Publisher steering_publisher = nh.advertise<std_msgs::Int16>("/AutoNOMOS_mini/manual_control/steering", 1);

  //std_msgs::Int16 velocity_message;
  //std_msgs::Int16 steering_message;
  
  int cont=0;

  ros::Rate r(5);
  sleep(2);
  while(nh.ok()){
  /*
    if(cont>400){
    	cont=0;
    }
    */
    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();
    last_time = current_time;
    
    movimiento(cont);
    ROS_INFO_STREAM("Contador " << cont);
    /*
    velocity_message.data=-100;
    steering_message.data=90;
    velocity_publisher.publish(velocity_message);
    steering_publisher.publish(steering_message);
    ROS_INFO_STREAM("Velocidad: " << velocity_message << " Steering: " << steering_message);
    sleep(10);
    
    velocity_message.data=-80;
    steering_message.data=30;
    velocity_publisher.publish(velocity_message);
    steering_publisher.publish(steering_message);
    ROS_INFO_STREAM("Velocidad: " << velocity_message << " Steering: " << steering_message);
    sleep(15);
 	*/
    cont=cont+1;	  
    r.sleep();
  }
}
