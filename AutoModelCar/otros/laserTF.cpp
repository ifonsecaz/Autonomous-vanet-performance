#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h> 
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>

#include <vector>

float angle_min=0.0;
float angle_max=0.0;
float angle_increment=0.0;
float time_increment=0.0;
float scan_time=0.0;
float range_min=0.0;
float range_max=0.0;
std::vector<float> ranges;
std::vector<float> intensities;

double x;
double y;
double th;

//Paso los datos; 
//Transformados
//const sensor_msgs::LaserScan::ConstPtr& msg
void laserCallback(const sensor_msgs::LaserScan& msg)
{
  angle_min=msg.angle_min;
  angle_max=msg.angle_max;
  angle_increment=msg.angle_increment;
  time_increment=msg.time_increment;
  scan_time=msg.scan_time;
  range_min=msg.range_min;
  range_max=msg.range_max;
  ranges=msg.ranges;
  intensities=msg.intensities;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "laser_publisher");
  ros::NodeHandle n("~");
  
  //std::string file_name,model_car_twist,model_car_yaw,steering_command,steering_feedback;
  //n.param<std::string>("file_name",file_name,"/cfg/SteerAngleActuator.xml");
  //n.param<std::string>("model_car_twist",model_car_twist,"twist");
  //n.param<std::string>("model_car_yaw",model_car_yaw,"yaw");
  //n.param<std::string>("steering_command",steering_command,"manual_control/steering");
  //n.param<std::string>("steering_feedback",steering_feedback,"steering_angle");

  //n.param("initial_x",initial_x,2.5);
  //n.param("initial_y",initial_y,0.5);
  //n.param("initial_yaw",initial_yaw,0.0);
  //n.param("bicycle_model",bicycle_model,false);
  //n.param("servo_with_feedback",servo_with_feedback,false);
  //n.param("publish_tf",publish_tf,false);
  
  //Valores iniciales
  x=0.15;
  y=0.0;
  th=0.0;

  ROS_INFO_STREAM("Initial_x:" << x << " m, initial_y: " << y << " m, initial_yaw: " << th << " radians");

  ros::Publisher laser_pub = n.advertise<sensor_msgs::LaserScan>("laserTF", 1); //o point_cloud
  ros::Subscriber laser_sub = n.subscribe("/scan", 10, laserCallback);

  tf::TransformBroadcaster laser_broadcaster;
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(1000.0);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //a quaternion created from yaw
    geometry_msgs::Quaternion laser_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    
	
	//Broadcaster
	geometry_msgs::TransformStamped laser_trans;
	laser_trans.header.stamp = current_time;
	laser_trans.header.frame_id = "base_laser";
	laser_trans.child_frame_id = "base_link"; //odom
	laser_trans.transform.translation.x = x; //x y no cambian
	laser_trans.transform.translation.y = y;
	laser_trans.transform.translation.z = 0.0;
	laser_trans.transform.rotation = laser_quat;

	//send the transform
	laser_broadcaster.sendTransform(laser_trans);


	//Listener
    //next, we'll publish the odometry message over ROS
    sensor_msgs::LaserScan tflaser;
    tflaser.header.stamp = current_time;
    tflaser.header.frame_id = "base_laser";
	
	tflaser.angle_min=angle_min;
	tflaser.angle_max=angle_max;
	tflaser.angle_increment=angle_increment;
	tflaser.time_increment=time_increment;
	tflaser.scan_time=scan_time;
	tflaser.range_min=range_min;
	tflaser.range_max=range_max;
  	tflaser.ranges=ranges;
	tflaser.intensities=intensities;

    //tflaser.child_frame_id = "base_link";  //odom, no tiene ese atributo

    //publish the message
    laser_pub.publish(tflaser);

    last_time = current_time;
    r.sleep();
  }
}

