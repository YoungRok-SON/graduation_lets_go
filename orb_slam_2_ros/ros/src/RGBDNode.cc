#include "RGBDNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc > 1) {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    ros::NodeHandle node_handle;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    image_transport::ImageTransport image_transport (node_handle);

    // RGBD Node class
    RGBDNode node (ORB_SLAM2::System::RGBD, node_handle, image_transport);

    // Node init
    node.Init();

    // Node loop. 주기는 따로 설정하나 봄.
    ros::spin();

    ros::shutdown();

    return 0;
}


RGBDNode::RGBDNode (const ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport) : Node (sensor, node_handle, image_transport) {
  rgb_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/rgb/image_raw", 1);
  depth_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/depth_registered/image_raw", 1);
  pointcloud_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2> (node_handle, "/camera/depth/color/points", 1);
  camera_info_topic_ = "/camera/rgb/camera_info";

  sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *rgb_subscriber_, *depth_subscriber_, *pointcloud_subscriber_);
  sync_->registerCallback(boost::bind(&RGBDNode::ImageCallback, this, _1, _2, _3));
}


RGBDNode::~RGBDNode () {
  delete rgb_subscriber_;
  delete depth_subscriber_;
  delete sync_;
}


void RGBDNode::ImageCallback (const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD, const sensor_msgs::PointCloud2ConstPtr& msgPointCloud) 
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try {
      // ROS에서 제공하는 cv_bridge를 이용하여 ROS 메시지를 OpenCV 메시지로 변환
      cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  } catch (cv_bridge::Exception& e) 
  {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  cv_bridge::CvImageConstPtr cv_ptrD;
  try 
  {
    // ROS에서 제공하는 cv_bridge를 이용하여 ROS 메시지를 OpenCV 메시지로 변환
    cv_ptrD = cv_bridge::toCvShare(msgD);
  } catch (cv_bridge::Exception& e) 
  {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  pcl::PointCloud<PointT>::Ptr pointcloud(new pcl::PointCloud<PointT>());
  try
  {
    pcl::fromROSMsg(*msgPointCloud, *pointcloud);
  }
  catch(const pcl::PCLException& e)
  {
    ROS_ERROR(" PCL Pointcloud expection: %s", e.what());
  }
  
  current_frame_time_ = msgRGB->header.stamp;

  // Pass the image to the SLAM system to track the camera pose.
  orb_slam_->TrackRGBDP(cv_ptrRGB->image, cv_ptrD->image, pointcloud, cv_ptrRGB->header.stamp.toSec());

  // Update the map visualization
  Update (); // Node Class의 Update 함수 호출
}
