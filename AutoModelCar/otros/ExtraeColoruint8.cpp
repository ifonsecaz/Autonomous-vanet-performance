#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h> 
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <stdint.h>

#include <vector>
std::vector<double> data;
std::vector<uint8_t> rV;
std::vector<uint8_t> gV;      
std::vector<uint8_t> bV;

//ros::NodeHandle nh;
ros::Publisher red_pub;
ros::Publisher green_pub;
ros::Publisher blue_pub;
//ros::Subscriber sub



//Transformación de los datos recibidos 
void cameraCallback(const std_msgs::Float64MultiArray& msg)
{
  ros::NodeHandle nh("~");
  data=msg.data;
  int size=data.size();
  float x;
  rV.clear();
  gV.clear();	
  bV.clear();
  uint32_t rgb;
  uint16_t r;
  uint16_t g; 
  uint16_t b;
  red_pub = nh.advertise<std_msgs::UInt8MultiArray>("/red", 1); 
  green_pub = nh.advertise<std_msgs::UInt8MultiArray>("/green", 1); 
  blue_pub = nh.advertise<std_msgs::UInt8MultiArray>("/blue", 1);

  for (int i = 1; i <= size; i++){
	x=float(data[i]);
	rgb = *reinterpret_cast<int*>(&x);
	r = (rgb >> 16) & 0x0000ff;
	g = (rgb >> 8)  & 0x0000ff;
	b = (rgb)       & 0x0000ff;
	rV.push_back(r);
	gV.push_back(g);
	bV.push_back(b);
  }
    std_msgs::UInt8MultiArray red;
    std_msgs::UInt8MultiArray green;
    std_msgs::UInt8MultiArray blue;
    red.data=rV;
    green.data=gV;
    blue.data=bV;

    red_pub.publish(red);
    green_pub.publish(green);
    blue_pub.publish(blue);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "splitColors_publisher");

  ros::NodeHandle nh("~");
  //  ros::Publisher red_pub;
  //ros::Publisher green_pub;
  //ros::Publisher blue_pub;
  ros::Subscriber camera_sub = nh.subscribe("/argb", 10, cameraCallback);

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(5.0);
  while(nh.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();
    last_time = current_time;
    r.sleep();
  }
}

