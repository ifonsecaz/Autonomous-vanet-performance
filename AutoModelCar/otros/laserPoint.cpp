#include <pointcloud_to_laserscan/laserscan_to_pointcloud_nodelet.h>
#include <sensor_msgs/LaserScan.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <string>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

namespace pointcloud_to_laserscan
{
LaserScanToPointCloudNodelet::LaserScanToPointCloudNodelet()
{
}

void LaserScanToPointCloudNodelet::onInit()
{
  boost::mutex::scoped_lock lock(connect_mutex_);
  private_nh_ = getPrivateNodeHandle();

  private_nh_.param<std::string>("target_frame", target_frame_, "");
  private_nh_.param<double>("transform_tolerance", transform_tolerance_, 0.01);

  int concurrency_level = private_nh_.param("concurrency_level", concurrency_level);

  // Check if explicitly single threaded, otherwise, let nodelet manager dictate thread pool size
  if (concurrency_level == 1)
  {
    nh_ = getNodeHandle();
  }
  else
  {
    nh_ = getMTNodeHandle();
  }

  // Only queue one pointcloud per running thread
  if (concurrency_level > 0)
  {
    input_queue_size_ = static_cast<unsigned int>(concurrency_level);
  }
  else
  {
    input_queue_size_ = boost::thread::hardware_concurrency();
  }

  // if pointcloud target frame specified, we need to filter by transform availability
  if (!target_frame_.empty())
  {
    tf2_.reset(new tf2_ros::Buffer());
    tf2_listener_.reset(new tf2_ros::TransformListener(*tf2_));
    message_filter_.reset(new MessageFilter(sub_, *tf2_, target_frame_, input_queue_size_, nh_));
    message_filter_->registerCallback(boost::bind(&LaserScanToPointCloudNodelet::scanCallback, this, _1));
    message_filter_->registerFailureCallback(boost::bind(&LaserScanToPointCloudNodelet::failureCallback, this, _1, _2));
  }
  else  // otherwise setup direct subscription
  {
    sub_.registerCallback(boost::bind(&LaserScanToPointCloudNodelet::scanCallback, this, _1));
  }

  pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("cloud", 10, boost::bind(&LaserScanToPointCloudNodelet::connectCb, this),
                                              boost::bind(&LaserScanToPointCloudNodelet::disconnectCb, this));
}

void LaserScanToPointCloudNodelet::connectCb()
{
  boost::mutex::scoped_lock lock(connect_mutex_);
  if (pub_.getNumSubscribers() > 0 && sub_.getSubscriber().getNumPublishers() == 0)
  {
    NODELET_INFO("Got a subscriber to cloud, starting laserscan subscriber");
    sub_.subscribe(nh_, "scan_in", input_queue_size_);
  }
}

void LaserScanToPointCloudNodelet::disconnectCb()
{
  boost::mutex::scoped_lock lock(connect_mutex_);
  if (pub_.getNumSubscribers() == 0)
  {
    NODELET_INFO("No subscibers to cloud, shutting down subscriber to laserscan");
    sub_.unsubscribe();
  }
}

void LaserScanToPointCloudNodelet::failureCallback(const sensor_msgs::LaserScanConstPtr& scan_msg,
                                                   tf2_ros::filter_failure_reasons::FilterFailureReason reason)
{
  NODELET_WARN_STREAM_THROTTLE(1.0, "Can't transform laserscan from frame " << scan_msg->header.frame_id << " to "
                                                                            << message_filter_->getTargetFramesString()
                                                                            << " at time " << scan_msg->header.stamp
                                                                            << ", reason: " << reason);
}

void LaserScanToPointCloudNodelet::scanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg)
{
  sensor_msgs::PointCloud2Ptr scan_cloud;
  scan_cloud.reset(new sensor_msgs::PointCloud2);
  projector_.projectLaser(*scan_msg, *scan_cloud);

  // Transform cloud if necessary
  if (!target_frame_.empty() && scan_cloud->header.frame_id != target_frame_)
  {
    try
    {
      *scan_cloud = tf2_->transform(*scan_cloud, target_frame_);
    }
    catch (tf2::TransformException& ex)
    {
      NODELET_ERROR_STREAM("Transform failure: " << ex.what());
      return;
    }
  }
  pub_.publish(*scan_cloud);
}
}