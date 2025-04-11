#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h> 
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/PointField.h>

#include <vector>

int height=0;
int width=0;
std::vector<sensor_msgs::PointField_<std::allocator<void> >,std::allocator<sensor_msgs::PointField_<std::allocator<void> > > > fields;
bool is_bigendian=false;
int point_step=0;
int row_step=0;
std::vector<unsigned char> data;
bool is_dense=false;

double x;
double y;
double thR;
double thP;
double thY;

//Transformación de los datos recibidos por el sensor
void cameraCallback(const sensor_msgs::PointCloud2& msg)
{
  height=msg.height;
  width=msg.width;
  fields=msg.fields;
  is_bigendian=msg.is_bigendian;
  point_step=msg.point_step;
  row_step=msg.row_step;
  data=msg.data;
  is_dense=msg.is_dense;  
  
}
/*
//Tipo imageptr
//Contenido
//std_msgs/Header header
//uint32 height
//uint32 width
//string encoding
//uint8 is_bigendian
//uint32 step
//uint8[] data

void cameraCallback(const sensor_msgs::Image& msg)
{
	
  height=msg.height; 
  width=msg.width;
  fields=msg.fields;
  is_bigendian=msg.is_bigendian;
  point_step=msg.point_step; 
  row_step=step;
  data=msg.data; //row_step*height
  is_dense=false;
  
}
*/
int main(int argc, char** argv){
  ros::init(argc, argv, "camera_publisher");
  ros::NodeHandle n("~");

  //Ubicación del laser con respecto al centro del carro (base_link)
  //No se toma en cuenta z
  //Valores iniciales
  x=0.15;
  y=0.0;
  thR=1.57;
  thY=0.0;
  thP=1.57;
 
  //Imprime la posición
  ROS_INFO_STREAM("Posicion camera: Initial_x:" << x << " m, initial_y: " << y << " m, initial_yaw: " << thR << "roll radians" << thP << "pitch radians" << thY << "yaw radians");

  //El  nodo publica dos mensajes, la transformada y los puntos transformados
  ros::Publisher camera_pub = n.advertise<sensor_msgs::PointCloud2>("cameraTF", 1); //tipo PointCloud2 o imagePtr
  //Se suscribe a la camara
  ros::Subscriber camera_sub = n.subscribe("/depth/points", 10, cameraCallback);

  //Publica el frame tipo tf2
  tf::TransformBroadcaster camera_broadcaster;
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(100.0); //Corregir
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //a quaternion created from yaw
    geometry_msgs::Quaternion camera_quat = tf::createQuaternionMsgFromRollPitchYaw(thR,thP,thY);

    //first, we'll publish the transform over tf
	
	//Broadcaster
	geometry_msgs::TransformStamped camera_trans;
	camera_trans.header.stamp = current_time;
	camera_trans.header.frame_id = "base_camera"; //Nombre del frame
	//Frame respecto al cual se publica las coordenadas
	//De base_link sale el frame odom y base_laser
	camera_trans.child_frame_id = "base_link"; 
	camera_trans.transform.translation.x = x; //Ubicación del frame
	camera_trans.transform.translation.y = y;
	camera_trans.transform.translation.z = 0.0;
	camera_trans.transform.rotation = camera_quat;

	//send the transform
	camera_broadcaster.sendTransform(camera_trans);


	//Listener
    //next, we'll publish the odometry message over ROS
    sensor_msgs::PointCloud2 tfcamera;
    tfcamera.header.stamp = current_time;
    tfcamera.header.frame_id = "base_camera"; //Nombre del frame al que se publican
	
	//Se copian los datos de la camara de profundidad
	tfcamera.height=height;
	tfcamera.width=width;
        tfcamera.fields=fields;
	tfcamera.is_bigendian=is_bigendian;
	tfcamera.point_step=point_step;
	tfcamera.row_step=row_step;
	tfcamera.data=data;
	tfcamera.is_dense=is_dense;

    //publish the message
    camera_pub.publish(tfcamera);

    last_time = current_time;
    r.sleep();
  }
}

