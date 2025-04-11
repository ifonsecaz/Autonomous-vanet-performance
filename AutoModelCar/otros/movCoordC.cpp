#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16.h>
#include <stdint.h>
#include <vector>
#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>

//ros::NodeHandle nh;
ros::Publisher velocity_publisher;
ros::Publisher steering_publisher;
//ros::Subscriber sub

int x = 0;
int y = 0;
int z = 0;
double theta = 1.57;
double xgoal;
double ygoal;
double zgoal;
double thetagoal;

void movimiento(int x, int y);

void poseCallback(const nav_msgs::Odometry& msg)
{
  xgoal=msg.pose.pose.position.x;
  ygoal=msg.pose.pose.position.y;
  double xq=msg.pose.pose.orientation.x;
  double yq=msg.pose.pose.orientation.y;
  double zq=msg.pose.pose.orientation.z;
  double wq=msg.pose.pose.orientation.w;
  //double theta=atan(2*(wq*zq+xq*yq),(1-2*(pow(yq,2)+pow(zq,2))));
  movimiento(0,0);
}

void movimiento (int x, int y){
        ros::NodeHandle nh("~");
        velocity_publisher = nh.advertise<std_msgs::Int16>("/manual_control/speed", 1);
        //velocity_publisher = nh.advertise<std_msgs::Int32>("/manual_control/speed", 1);
        steering_publisher = nh.advertise<std_msgs::Int16>("/manual_control/steering", 1);


        std_msgs::Int16 velocity_message;
        //std_msgs::Int32 velocity_message;
        std_msgs::Int16 steering_message;

        double kh = 1;
        double dtheta;
        double desired_angle_goal;
        desired_angle_goal=atan2(ygoal-y, xgoal-x);
        //rads
        if (desired_angle_goal < 0){
                desired_angle_goal = desired_angle_goal + 2*M_PI;
        }
        else{
                desired_angle_goal = desired_angle_goal;
        }

        if (theta<0){
                theta=2*M_PI+theta;
        }
        dtheta = desired_angle_goal - theta;

        if (dtheta < -1.57 && dtheta >-3.14){
                dtheta = -1.57;
        }
        else{
                if (dtheta >1.57 && dtheta <3.14){
                        dtheta = 1.57;
                }
                        else{
                                if (dtheta <-3.14){
                                        dtheta = 6.2831+dtheta;
                                }
                                else{
                                        if (dtheta > 3.14){
                                                dtheta = dtheta-6.2831;
                                        }
                                }
                        }
        }
        //Velocidad
        double ka = 5.0;

        double distance = abs(sqrt((pow((xgoal-(double)x),2)+pow((ygoal-(double)y),2))));
        double linear_speed = -1*ka * distance;

        if (linear_speed > 1000){
        linear_speed=1000;
        }
        else{
            if (linear_speed < -1200){
                linear_speed = -1200;
            }
            else{
                if (linear_speed>-75 && linear_speed<0){
                    linear_speed=-75;
                }
            }
        }
        if (distance < 30){
            linear_speed=0.0;
        }


    //steering_message.Int16=kh*dtheta
        int steeringD=static_cast<int>(kh*((dtheta*360.00/6.2831)));
        int aux2=90+(steeringD)*90/30;
        ROS_INFO_STREAM("S: " << steeringD);
        if (aux2<0){
                aux2=0;
        }
        else{
            if (aux2>180){
                aux2=180;
            }
        }
        steering_message.data=aux2;


        //steering_publisher.publish(steering_message);

        //velocity_message.Int16 = linear_speed
        velocity_message.data=static_cast<int>(linear_speed);

        ROS_INFO_STREAM("Velocidad: " << linear_speed << " Steering: " << aux2);

        //velocity_publisher.publish(velocity_message);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "controlCoord_publisher");

  ros::NodeHandle nh("~");
  ros::Subscriber meta_sub = nh.subscribe("/meta", 10, poseCallback);

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(2.0);
  while(nh.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();
    last_time = current_time;
    r.sleep();
  }
}