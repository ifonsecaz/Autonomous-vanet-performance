#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>



double x = 0.0;
double y = 0.0;
double th = 0.0;

double head=0.0;
double last_head=0.0;
double initial_head=0.0;
double vx = 0.0;
double vy = 0.0;
double vth = 0.0;
bool init=false;
ros::Time current_time_twist, last_time_twist;
void twistCallback(const geometry_msgs::Twist& msg)
{
  float vx_=round(msg.linear.x / (9.54929659643*5.5))*(0.031);// 9.54929659643 rpm = 1rad/s and gear ratio: 6  and the wheel Radius 0.031 meter
  vx = roundf(vx_ * -100) / 100;  /* Result: 37.78 */ ///0,00054 -> 0.00081
}
void headingCallback(const std_msgs::Float32& msg)
{
  if (init==false)
  {
    init=true;
    head=msg.data* (3.14/180.0); //rad
    initial_head=head;
    vth=0.0;
    current_time_twist = ros::Time::now();
    last_time_twist=current_time_twist;
    th=msg.data* (3.14/180.0);
  }
  else
  {
    // last_time_twist=current_time_twist;
    // current_time_twist = ros::Time::now();
    //last_head=head;
    head=msg.data* (3.14/180.0); //rad
    // double dt_twist = (current_time_twist - last_time_twist).toSec();
    double delta_head=head-initial_head;
    if (delta_head>3.14)
      delta_head=delta_head-6.28;
    else if (delta_head<-3.14)
      delta_head=delta_head+6.28;
    // float vth_=0.0;
    // vth_= delta_head/dt_twist;//100
    // vth = roundf(vth_ * 1000) / 1000;  /* Result: 37.78 */
    th = roundf(delta_head * 100) / 100;
  }
  
}
int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
	
  //Similar al nodo odometry_publisher (base_link --> odom)
  //Únicamente publica el frame inverso (odom --> base_link)

  //Calcula la odometría igual que el otro nodo
  //Se debe cambiar para que lea odom
  ros::Subscriber twist_sub = n.subscribe( "motor_control/twist", 10, twistCallback);
  ros::Subscriber theta_sub = n.subscribe( "model_car/yaw", 10, headingCallback);//degree

  tf::TransformBroadcaster odom_broadcaster;
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(1000.0);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    dt = roundf(dt * 10000) / 10000;
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    // double delta_th = vth * 0.01 ; //* dt;
    x += delta_x;
    y += delta_y;
    //th += delta_th;

    // ROS_INFO("t %0.6f",dt);

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);


    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
	//Los frames se invierten, ahora el nodo padre es odom
    odom_trans.header.frame_id = "base_link";
    odom_trans.child_frame_id = "odom";
	
	//Ubicación
	//Inversa de la transformada con cuaterniones es: (-q^-1*t,q^-1)
	//t se pasa a negativo -t
    odom_trans.transform.translation.x = -x;
    odom_trans.transform.translation.y = -y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
 
	//Inversa de la rotación de un cuaternion, es el negativo del parámetro w
	//q^-1
    odom_trans.transform.rotation.w = -1*(odom_trans.transform.rotation.w);	

    //tf::Transform transform;
    //tf::transformMsgToTF(odom_trans, transform);
    //geometry_msgs::Transform inverted_transform_msg;
    //tf::transformTFToMsg(transform.inverse(), inverted_transform_msg);

    //send the transform
    odom_broadcaster.sendTransform(odom_trans)

    last_time = current_time;
    r.sleep();
  }
}

