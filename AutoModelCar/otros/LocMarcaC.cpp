#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
//#include <sensor_msgs/point_cloud2.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
//#include <sensor_msgs/point_cloud2.h>
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
//#include <pcl_ros/pointcloud_to_pcd.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <boost/foreach.hpp>

//ros::NodeHandle nh;
ros::Publisher meta_publisher;
ros::Publisher rgb_publisher;

//ros::Subscriber sub

void localizaMarca();

float x = 0;
float y = 0;
float z = 0;
int punto = 0;
int height;
int width;
std::vector<float> xA; // o double
std::vector<float> yA;
std::vector<float> zA;
std::vector<uint8_t> r;
std::vector<uint8_t> g;
std::vector<uint8_t> b;
sensor_msgs::PointCloud2 msg2;

void camaraRGBCallback(const sensor_msgs::PointCloud2& msg)
{
  msg2=msg;
  //o msg.data.size
}


void camaraCallback(const sensor_msgs::PointCloud2& msg)
{
        ros::NodeHandle nh("~");
        rgb_publisher = nh.advertise<std_msgs::UInt8MultiArray>("/argb", 1);
        //pcl::PointCloud<pcl::PointXYZRGB> point_cloud;

        //pcl::fromPCLPointCloud2( msg, point_cloud);

        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(msg,pcl_pc2);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pt_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::fromPCLPointCloud2(pcl_pc2,*pt_cloud);

        //std::vector<float> argb;
        xA.clear();
        yA.clear();
        zA.clear();
        r.clear();
        g.clear();
        b.clear();
        //argb.clear();
        height=msg.height;
        width=msg.width;
        for(int i = 0; i <  height*width; ++i){
                xA.push_back(pt_cloud->points[i].x);
                yA.push_back(pt_cloud->points[i].y);
                zA.push_back(pt_cloud->points[i].z);
                r.push_back(pt_cloud->points[i].r); // o rgb
                g.push_back(pt_cloud->points[i].g);
                b.push_back(pt_cloud->points[i].b);
        }
        std_msgs::UInt8MultiArray color;
        color.data=r;
        rgb_publisher.publish(color);
        localizaMarca();
}

//UInt16MultiArray
//void colorCallback(const std_msgs::UInt8MultiArray& msg){
    //r=(uint8_t)msg.data;
//    r=msg.data;
//      localizaMarca();
//}

void localizaMarca(){
        ros::NodeHandle nh("~");
        meta_publisher = nh.advertise<nav_msgs::Odometry>("/meta", 1);
        nav_msgs::Odometry odometry_message;
        int num=r.size(); //307200, 480 filas, 640 columnas
        //0, 480...
        std::vector<uint8_t> rV;
        std::vector<uint8_t> sumaRow;
        std::vector<uint8_t> sumaCol;
        //Se aplica el umbral y suma total
        int total=0;
        for (int i=0; i<num; i++){
                if(r.at(i)<=5){
                        total=total+1;
                        rV.push_back(1);
                }
                else{
                        rV.push_back(0);
                }
        }
        int mitad=total/2;
        //Suma columnas
        int i=0;
        int aux=0;
        int j=0;
        while(i<width&&aux<mitad){
                j=0;
                while(j<height&&aux<mitad){
                        aux=aux+rV.at(j*width+i);
                        j++;
                }
                i++;
        }
        int col=i;

        //Suma filas
        i=0;
        aux=0;
        while(i<height&&aux<mitad){
                j=0;
                while(j<width&&aux<mitad){
                        aux=aux+rV.at(i*width+j);
                        j++;
                }
                i++;
        }
    int row=i;

        if (row==height){
        row=0;
        }
    if (col==width){
        col=0;
        }
    int punto=col+row*width;
        x=xA.at(punto);
        y=yA.at(punto);
        z=zA.at(punto);
        if(punto!=0){
        odometry_message.pose.pose.position.x=100*x; //-8.764
        odometry_message.pose.pose.position.y=100*z; //Distancia a la que pararse
        odometry_message.pose.pose.position.z=0;
        odometry_message.pose.pose.orientation.x=0;
        odometry_message.pose.pose.orientation.y=0;
        odometry_message.pose.pose.orientation.z=0;
        odometry_message.pose.pose.orientation.w=0;
                meta_publisher.publish(odometry_message);
        }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "meta_publisher");

  ros::NodeHandle nh("~");
  ros::Subscriber camara_sub = nh.subscribe("/depth/points", 10, camaraRGBCallback);
  //ros::Subscriber color_sub = nh.subscribe("/red", 10, colorCallback);

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(2.0);
  sleep(2);
  while(nh.ok()){
    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();
    last_time = current_time;
        camaraCallback(msg2);
    r.sleep();
  }
}