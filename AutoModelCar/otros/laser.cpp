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

//Transformación de los datos recibidos por el sensor
//En este caso únicamente se copian los datos, se genera el mismo mensaje laserScan
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

  //Ubicación del laser con respecto al centro del carro (base_link)
  //No se toma en cuenta z
  //Valores iniciales
  x=0.15;
  y=0.0;
  th=0.0;
 
  //Imprime la posición
  ROS_INFO_STREAM("Posicion laser: Initial_x:" << x << " m, initial_y: " << y << " m, initial_yaw: " << th << " radians");

  //El  nodo publica dos mensajes, la transformada y los puntos transformados, en este case de tipo LaserScan
  ros::Publisher laser_pub = n.advertise<sensor_msgs::LaserScan>("laserTF", 1); //o point_cloud
  //Se suscribe al lidar
  ros::Subscriber laser_sub = n.subscribe("/scan", 10, laserCallback);

  //Publica el frame tipo tf2
  tf::TransformBroadcaster laser_broadcaster;
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(1000.0); //Corregir
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //a quaternion created from yaw
    geometry_msgs::Quaternion laser_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
	
	//Broadcaster
	geometry_msgs::TransformStamped laser_trans;
	laser_trans.header.stamp = current_time;
	laser_trans.header.frame_id = "base_laser"; //Nombre del frame
	//Frame respecto al cual se publica las coordenadas
	//De base_link sale el frame odom y base_laser
	laser_trans.child_frame_id = "base_link"; 
	laser_trans.transform.translation.x = x; //Ubicación del frame
	laser_trans.transform.translation.y = y;
	laser_trans.transform.translation.z = 0.0;
	laser_trans.transform.rotation = laser_quat;

	//send the transform
	laser_broadcaster.sendTransform(laser_trans);


	//Listener
    //next, we'll publish the odometry message over ROS
    sensor_msgs::LaserScan tflaser;
    tflaser.header.stamp = current_time;
    tflaser.header.frame_id = "base_laser"; //Nombre del frame al que se publican
	
	//Se copian los datos del lidar
	tflaser.angle_min=angle_min;
	tflaser.angle_max=angle_max;
	tflaser.angle_increment=angle_increment;
	tflaser.time_increment=time_increment;
	tflaser.scan_time=scan_time;
	tflaser.range_min=range_min;
	tflaser.range_max=range_max;
  	tflaser.ranges=ranges;
	tflaser.intensities=intensities;

    //tflaser.child_frame_id = "base_link";  //LaserScan no tiene ese atributo

    //publish the message
    laser_pub.publish(tflaser);

    last_time = current_time;
    r.sleep();
  }
}

