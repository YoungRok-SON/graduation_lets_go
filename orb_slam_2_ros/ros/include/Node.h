/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ORBSLAM2_ROS_NODE_H_
#define ORBSLAM2_ROS_NODE_H_

#include <vector>
#include <ros/ros.h>
#include <ros/time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_reconfigure/server.h>
#include <orb_slam2_ros/dynamic_reconfigureConfig.h>

#include "orb_slam2_ros/SaveMap.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Bool.h>

#include "System.h"

typedef pcl::PointXYZRGB PointT;

class Node
{
  public:
    // Node constructor
    Node (ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport); // RGBDNode 클래스가 생성될 때 호출되는 생성자
    ~Node (); // RGBD의 소멸자가 소멸되고 나서 호출되는 소멸자
    // RGBDNode.cc의 메인에서 호출되어 Node 클래스의 초기화를 수행하는 함수
    void Init ();

  protected:
    // Node update
    void Update (); // 매 Callback의 마지막에 호출되는 함수
    ORB_SLAM2::System* orb_slam_;  // ORB_SLAM2의 System 클래스의 포인터
    ros::Time current_frame_time_; // 현재 프레임의 시간

    std::string camera_info_topic_;

  private:
    // 맵 포인트를 퍼블리시하는 함수
    void PublishMapPoints (std::vector<ORB_SLAM2::MapPoint*> map_points); 
    // 위치를 퍼블리시하는 함수 (Transform)
    void PublishPositionAsTransform (cv::Mat position); 
    // 위치를 퍼블리시하는 함수 (PoseStamped)
    void PublishPositionAsPoseStamped(cv::Mat position); 
    // GBA 상태를 퍼블리시하는 함수 
    void PublishGBAStatus (bool gba_status); 
    // 렌더링된 이미지를 퍼블리시하는 함수 (ORB_SLAM2의 Viewer 클래스에서 렌더링된 이미지를 받아서 퍼블리시)
    void PublishRenderedImage (cv::Mat image); 
    // dynamic_reconfigure의 파라미터가 변경되었을 때 호출되는 콜백 함수
    void ParamsChangedCallback(orb_slam2_ros::dynamic_reconfigureConfig &config, uint32_t level); 
    // 맵을 저장하는 서비스 서버
    bool SaveMapSrv (orb_slam2_ros::SaveMap::Request &req, orb_slam2_ros::SaveMap::Response &res); 
    // ORB_SLAM2의 ORBParameters 클래스를 초기화하는 함수
    void LoadOrbParameters (ORB_SLAM2::ORBParameters& parameters); 

    // initialization Transform listener
    boost::shared_ptr<tf2_ros::Buffer> tfBuffer;
    boost::shared_ptr<tf2_ros::TransformListener> tfListener;

    // 위치 인자를 받아서 Transform으로 변환하는 함수
    tf2::Transform TransformFromMat (cv::Mat position_mat);
    // Transform을 받아서 frame_in에서 frame_target으로 변환하는 함수
    tf2::Transform TransformToTarget (tf2::Transform tf_in, std::string frame_in, std::string frame_target);
    // 맵 포인트를 포인트 클라우드로 변환하는 함수
    sensor_msgs::PointCloud2 MapPointsToPointCloud (std::vector<ORB_SLAM2::MapPoint*> map_points); 

    // dynamic_reconfigure의 파라미터를 저장하는 변수
    dynamic_reconfigure::Server<orb_slam2_ros::dynamic_reconfigureConfig> dynamic_param_server_;

    image_transport::Publisher rendered_image_publisher_;
    ros::Publisher map_points_publisher_;
    ros::Publisher pose_publisher_;
    ros::Publisher status_gba_publisher_;

    ros::ServiceServer service_server_;

    std::string name_of_node_;
    ros::NodeHandle node_handle_;
    image_transport::ImageTransport image_transport_;

    ORB_SLAM2::System::eSensor sensor_;

    std::string map_frame_id_param_;
    std::string camera_frame_id_param_;
    std::string target_frame_id_param_;
    std::string map_file_name_param_;
    std::string voc_file_name_param_;
    bool load_map_param_;
    bool publish_pointcloud_param_;
    bool publish_tf_param_;
    bool publish_pose_param_;
    // Number of minimal observations a key point must have to be published in the point cloud. 
    // This doesn't influence the behavior of the SLAM itself at all.
    int min_observations_per_point_;
};

#endif //ORBSLAM2_ROS_NODE_H_
