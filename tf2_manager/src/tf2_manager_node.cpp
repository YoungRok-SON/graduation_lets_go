
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf2_manager_node");
  ros::NodeHandle nh;

  static tf2_ros::StaticTransformBroadcaster static_broadcaster;

  // Depth to camera_link transform
  geometry_msgs::TransformStamped depth_to_camera_link;
  depth_to_camera_link.header.frame_id = "camera_link";
  depth_to_camera_link.child_frame_id = "camera_depth_optical_frame";
  depth_to_camera_link.transform.translation.x = 0.0;
  depth_to_camera_link.transform.translation.y = 0.0;
  depth_to_camera_link.transform.translation.z = 0.0;
  depth_to_camera_link.transform.rotation.x = -0.5;
  depth_to_camera_link.transform.rotation.y =  0.5;
  depth_to_camera_link.transform.rotation.z = -0.5;
  depth_to_camera_link.transform.rotation.w =  0.5;

  // RGB to camera_link transform
  geometry_msgs::TransformStamped rgb_to_camera_link;
  rgb_to_camera_link.header.frame_id = "camera_link";
  rgb_to_camera_link.child_frame_id = "camera_color_optical_frame";
  rgb_to_camera_link.transform.translation.x = 0.0;
  rgb_to_camera_link.transform.translation.y = 0.0;
  rgb_to_camera_link.transform.translation.z = 0.0;
  rgb_to_camera_link.transform.rotation.x = -0.5;
  rgb_to_camera_link.transform.rotation.y =  0.5;
  rgb_to_camera_link.transform.rotation.z = -0.5;
  rgb_to_camera_link.transform.rotation.w =  0.5;

  // Camera_link to base_link transform
  geometry_msgs::TransformStamped camera_link_to_base_link;
  camera_link_to_base_link.header.frame_id = "base_link";
  camera_link_to_base_link.child_frame_id = "camera_link";
  camera_link_to_base_link.transform.translation.x = 0.0;
  camera_link_to_base_link.transform.translation.y = 0.0;
  camera_link_to_base_link.transform.translation.z = 0.0;
  camera_link_to_base_link.transform.rotation.x = 0.0;
  camera_link_to_base_link.transform.rotation.y = 0.0;
  camera_link_to_base_link.transform.rotation.z = 0.0;
  camera_link_to_base_link.transform.rotation.w = 1.0;

  ros::Rate rate(10.0);
  while (nh.ok())
  {
    depth_to_camera_link.header.stamp = ros::Time::now();
    rgb_to_camera_link.header.stamp = ros::Time::now();
    camera_link_to_base_link.header.stamp = ros::Time::now();

    static_broadcaster.sendTransform(depth_to_camera_link);
    static_broadcaster.sendTransform(rgb_to_camera_link);
    static_broadcaster.sendTransform(camera_link_to_base_link);

    rate.sleep();
  }

  return 0;
}
