// SPDX-License-Identifier: BSD-2-Clause

#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <pcl_ros/point_cloud.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Time.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <hdl_graph_slam/ros_utils.hpp>
#include <hdl_graph_slam/registrations.hpp>
#include <hdl_graph_slam/ScanMatchingStatus.h>

namespace hdl_graph_slam
{

  class ScanMatchingOdometryNodelet : public nodelet::Nodelet
  {
  public:
    typedef pcl::PointXYZI PointT;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ScanMatchingOdometryNodelet() {}
    virtual ~ScanMatchingOdometryNodelet() {}

    virtual void onInit()
    {
      NODELET_DEBUG("initializing scan_matching_odometry_nodelet...");
      nh = getNodeHandle();
      private_nh = getPrivateNodeHandle();

      initialize_params();

      if (private_nh.param<bool>("enable_imu_frontend", false))
      {
        msf_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/msf_core/pose", 1, boost::bind(&ScanMatchingOdometryNodelet::msf_pose_callback, this, _1, false));
        msf_pose_after_update_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/msf_core/pose_after_update", 1, boost::bind(&ScanMatchingOdometryNodelet::msf_pose_callback, this, _1, true));
      }

      points_sub         = nh.subscribe("/filtered_points", 256, &ScanMatchingOdometryNodelet::cloud_callback, this);
      read_until_pub     = nh.advertise<std_msgs::Header>("/scan_matching_odometry/read_until", 32);
      odom_pub           = nh.advertise<nav_msgs::Odometry>(published_odom_topic, 32);
      trans_pub          = nh.advertise<geometry_msgs::TransformStamped>("/scan_matching_odometry/transform", 32);
      status_pub         = private_nh.advertise<ScanMatchingStatus>("/scan_matching_odometry/status", 8);
      aligned_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/aligned_points", 32);
    }

  private:
    /**
     * @brief initialize parameters
     */
    void initialize_params()
    {
      auto &pnh = private_nh;
      published_odom_topic = private_nh.param<std::string>("published_odom_topic", "/odom");
      points_topic = pnh.param<std::string>("points_topic", "/velodyne_points");
      odom_frame_id = pnh.param<std::string>("odom_frame_id", "odom");
      robot_odom_frame_id = pnh.param<std::string>("robot_odom_frame_id", "robot_odom");

      // The minimum tranlational distance and rotation angle between keyframes.
      // If this value is zero, frames are always compared with the previous frame
      keyframe_delta_trans = pnh.param<double>("keyframe_delta_trans", 0.25);
      keyframe_delta_angle = pnh.param<double>("keyframe_delta_angle", 0.15);
      keyframe_delta_time = pnh.param<double>("keyframe_delta_time", 1.0);

      // Registration validation by thresholding
      transform_thresholding = pnh.param<bool>("transform_thresholding", false);
      max_acceptable_trans = pnh.param<double>("max_acceptable_trans", 1.0);
      max_acceptable_angle = pnh.param<double>("max_acceptable_angle", 1.0);

      // select a downsample method (VOXELGRID, APPROX_VOXELGRID, NONE)
      std::string downsample_method = pnh.param<std::string>("downsample_method", "VOXELGRID");
      double downsample_resolution = pnh.param<double>("downsample_resolution", 0.1);
      if (downsample_method == "VOXELGRID")
      {
        std::cout << "downsample: VOXELGRID " << downsample_resolution << std::endl;
        auto voxelgrid = new pcl::VoxelGrid<PointT>();
        voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
        downsample_filter.reset(voxelgrid);
      }
      else if (downsample_method == "APPROX_VOXELGRID")
      {
        std::cout << "downsample: APPROX_VOXELGRID " << downsample_resolution << std::endl;
        pcl::ApproximateVoxelGrid<PointT>::Ptr approx_voxelgrid(new pcl::ApproximateVoxelGrid<PointT>());
        approx_voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
        downsample_filter = approx_voxelgrid;
      }
      else
      {
        if (downsample_method != "NONE")
        {
          std::cerr << "warning: unknown downsampling type (" << downsample_method << ")" << std::endl;
          std::cerr << "       : use passthrough filter" << std::endl;
        }
        std::cout << "downsample: NONE" << std::endl;
        pcl::PassThrough<PointT>::Ptr passthrough(new pcl::PassThrough<PointT>());
        downsample_filter = passthrough;
      }

      // Registration cpp에서 GICP나 ICP, NGD_OMP와 같은 방법의
      // 클래스를 가져와 객체로 반환해줌
      // NDT_OMP에 대해 알고 싶으면 NDT_OMP 패키지 코드를 더 까봐야함.
      registration = select_registration_method(pnh); 
      // 지정한 방법을 가지고 있는 클래스 객체의 Ptr을 반환
      // ex) pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());

    }

    /**
     * @brief callback for point clouds
     * @param cloud_msg  point cloud msg
     */
    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)  // 이 노들렛은 왜 또 ROS형식으로 받는건지.. PCL로 바로 받으면 안되나?
    {
      if (!ros::ok()) // 아 Ctrl+C 잘먹게 하려고..
      {
        return;
      }

      pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
      pcl::fromROSMsg(*cloud_msg, *cloud); // 어차피 여기서 변환할 꺼 그냥 prefiltering처럼 받으면 안되나?

      Eigen::Matrix4f pose = matching(cloud_msg->header.stamp, cloud);  // 얘가 제일 중요한..
      publish_odometry(cloud_msg->header.stamp, cloud_msg->header.frame_id, pose); // Odometry 데이터 출력

      // In offline estimation, point clouds until the published time will be supplied -> ??
      std_msgs::HeaderPtr read_until(new std_msgs::Header());
      read_until->frame_id = points_topic;
      read_until->stamp = cloud_msg->header.stamp + ros::Duration(1, 0);
      read_until_pub.publish(read_until);

      read_until->frame_id = "/filtered_points"; 
      read_until_pub.publish(read_until);
      /* 이 코드의 의도는 특정한 주제에 대한 포인트 클라우드 데이터를 읽어오는 동안,
         다음 1초 동안의 데이터도 함께 읽어오라는 지시를 주는 것으로 보입니다.
         또한, 원본 포인트 클라우드 데이터와 필터링된 포인트 클라우드 데이터에 대한 정보를 각각 발행하는 것으로 보입니다. */
    }

    void msf_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose_msg, bool after_update)
    {
      if (after_update)
      {
        msf_pose_after_update = pose_msg;
      }
      else
      {
        msf_pose = pose_msg;
      }
    }

    /**
     * @brief downsample a point cloud
     * @param cloud  input cloud
     * @return downsampled point cloud
     */
    pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr &cloud) const
    {
      if (!downsample_filter)
      {
        return cloud;
      }

      pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
      downsample_filter->setInputCloud(cloud);
      downsample_filter->filter(*filtered);

      return filtered;
    }

    /**
     * @brief estimate the relative pose between an input cloud and a keyframe cloud
     * @param stamp  the timestamp of the input cloud
     * @param cloud  the input cloud
     * @return the relative pose between the input cloud and the keyframe cloud
     */
    Eigen::Matrix4f matching(const ros::Time &stamp, const pcl::PointCloud<PointT>::ConstPtr &cloud)
    {
      // Keyframe이 없을 때
      if (!keyframe) // 이 keyframe은 어디서 값을 넣어주는거지?
      {
        prev_time = ros::Time();
        prev_trans.setIdentity(); // prev_trans는 previous estimated transform from keyframe이라는데 keyframe으로부터 이전 프레임까지의 변환행렬을 저장하는 거겠지?
        keyframe_pose.setIdentity(); 
        keyframe_stamp = stamp; // 현시점 stamp를 카프레임 시간으로 지정
        keyframe = downsample(cloud); // downsampling을 하는데 .. 이미 prefiltering에서 필터링 진행하고 온 거 아닌가? 왜 또 하지? -> 필요한 경우 진행하고 roslaunch에 안하도록 지정되어 있음
        registration->setInputTarget(keyframe); // 지금 들어온 프레임을 키프레임으로 지정
        return Eigen::Matrix4f::Identity();
      }

      // Keyframe이 지정된 경우
      auto filtered = downsample(cloud);
      registration->setInputSource(filtered); // 정합에 들어온 포인트 클라우드 전달

      std::string msf_source;
      Eigen::Isometry3f msf_delta = Eigen::Isometry3f::Identity();
      
      // msf가 뭔지 모르겠음. IMU Preintegration해서 init pose로 사용하려는건가..?
      if (private_nh.param<bool>("enable_imu_frontend", false))
      {
        if (msf_pose && msf_pose->header.stamp > keyframe_stamp && msf_pose_after_update && msf_pose_after_update->header.stamp > keyframe_stamp)
        {
          Eigen::Isometry3d pose0 = pose2isometry(msf_pose_after_update->pose.pose);
          Eigen::Isometry3d pose1 = pose2isometry(msf_pose->pose.pose);
          Eigen::Isometry3d delta = pose0.inverse() * pose1;

          msf_source = "imu";
          msf_delta = delta.cast<float>();
        }
        else
        {
          std::cerr << "msf data is too old" << std::endl;
        }
      }
      else if (private_nh.param<bool>("enable_robot_odometry_init_guess", false) && !prev_time.isZero()) // 이것도 지금 안쓰는 걸로는 되어 있는데...
      {
        tf::StampedTransform transform;
        if (tf_listener.waitForTransform(cloud->header.frame_id, stamp, cloud->header.frame_id, prev_time, robot_odom_frame_id, ros::Duration(0)))
        {
          tf_listener.lookupTransform(cloud->header.frame_id, stamp, cloud->header.frame_id, prev_time, robot_odom_frame_id, transform);
        }
        else if (tf_listener.waitForTransform(cloud->header.frame_id, ros::Time(0), cloud->header.frame_id, prev_time, robot_odom_frame_id, ros::Duration(0)))
        {
          tf_listener.lookupTransform(cloud->header.frame_id, ros::Time(0), cloud->header.frame_id, prev_time, robot_odom_frame_id, transform);
        }

        if (transform.stamp_.isZero())
        {
          NODELET_WARN_STREAM("failed to look up transform between " << cloud->header.frame_id << " and " << robot_odom_frame_id);
        }
        else
        {
          msf_source = "odometry";
          msf_delta = tf2isometry(transform).cast<float>();
        }
      }

      pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());

      // 지정한 registration 방법으로 align 진행
      registration->align(*aligned, prev_trans * msf_delta.matrix()); // origin -> keyframe -> init guess 위치로 데이터 전달

      // 디버깅용으로 넣어놓은 듯 함. Scan matching 성능 체크하기 위한 함수로 보임
      publish_scan_matching_status(stamp, cloud->header.frame_id, aligned, msf_source, msf_delta);

      // 수렴하지 못하면 벌어지는 참사
      if (!registration->hasConverged())
      {
        NODELET_INFO_STREAM("scan matching has not converged!!");
        NODELET_INFO_STREAM("ignore this frame(" << stamp << ")");
        return keyframe_pose * prev_trans; // 여기서 이전에 성공한 registration이 있다면 keyframe_pose는 odom데이터와 같음
      }

      Eigen::Matrix4f trans = registration->getFinalTransformation();
      Eigen::Matrix4f odom = keyframe_pose * trans;

      if (transform_thresholding)  // Registration 결과에 대한 검사
      {
        Eigen::Matrix4f delta = prev_trans.inverse() * trans; // 이전 키프레임과 이번 정합 사이에 구한 변환 차이
        double dx = delta.block<3, 1>(0, 3).norm(); // 평행이동 변환
        double da = std::acos(Eigen::Quaternionf(delta.block<3, 3>(0, 0)).w()); // 회전변환의 변화값을 저장(w)

        if (dx > max_acceptable_trans || da > max_acceptable_angle) // 거리와 각도가 지정한 값보다 크면 // 무시
        {
          NODELET_INFO_STREAM("too large transform!!  " << dx << "[m] " << da << "[rad]");
          NODELET_INFO_STREAM("ignore this frame(" << stamp << ")");
          return keyframe_pose * prev_trans;
        }
      }
      
      // 위 조건을 통과하면 prev_trans와 stamp에 저장
      prev_time = stamp;
      prev_trans = trans;


      auto keyframe_trans = matrix2transform(stamp, keyframe_pose, odom_frame_id, "keyframe");
      keyframe_broadcaster.sendTransform(keyframe_trans);

      // 이거 여기서 왜 또 하는거지? -> 여기는 키프래임을 설정하기 위한 부분 이전은 registration을 검사하는 부분
      double delta_trans = trans.block<3, 1>(0, 3).norm();
      double delta_angle = std::acos(Eigen::Quaternionf(trans.block<3, 3>(0, 0)).w());
      double delta_time = (stamp - keyframe_stamp).toSec();
      // Keyframe 지정을 위한 검사 -> 지정한 길이보다 변환의 길이가 길면 그 측정값이 들어온 때에 Keyframe을 생성
      
      // ToDo 
      // RGB-D 센서와 같이 포인트 인지 거리가 짧은 센서들은 Keyframe_delta 값이 짧아야 registration이 잘 될 것으로 생각됨. -> 해보자
      // 아니면 registration을 할 때 keyframe 주변에 n개의 프레임을 매칭 데이터로 넘겨주는 식으로 변경이 되어야 매칭이 잘 될 것으로 생각됨. -> 위 결과에 따라 필요할 수도?
      if (delta_trans > keyframe_delta_trans || delta_angle > keyframe_delta_angle || delta_time > keyframe_delta_time)
      {
        keyframe = filtered;
        registration->setInputTarget(keyframe); // 바꾼다면 이 부분에 들어가는 pointcloud를 local map으로 주는 형식으로 되어야 할듯

        keyframe_pose = odom;
        keyframe_stamp = stamp;
        prev_time = stamp;
        prev_trans.setIdentity();
      }

      if (aligned_points_pub.getNumSubscribers() > 0)
      {
        pcl::transformPointCloud(*cloud, *aligned, odom);
        aligned->header.frame_id = odom_frame_id;
        aligned_points_pub.publish(*aligned);
      }

      return odom;
    }

    /**
     * @brief publish odometry for IMU integration, tf, and nav_msgs
     * @param stamp  timestamp
     * @param pose   odometry pose to be published
     */
    void publish_odometry(const ros::Time &stamp, const std::string &base_frame_id, const Eigen::Matrix4f &pose)
    {

      // 용도가 다 달라 대단해 Koide쨩

      // publish transform stamped for IMU integration
      geometry_msgs::TransformStamped odom_trans = matrix2transform(stamp, pose, odom_frame_id, base_frame_id);
      trans_pub.publish(odom_trans);

      // broadcast the transform over tf
      odom_broadcaster.sendTransform(odom_trans);

      // publish the transform
      nav_msgs::Odometry odom;
      odom.header.stamp = stamp;
      odom.header.frame_id = odom_frame_id;

      odom.pose.pose.position.x = pose(0, 3);
      odom.pose.pose.position.y = pose(1, 3);
      odom.pose.pose.position.z = pose(2, 3);
      odom.pose.pose.orientation = odom_trans.transform.rotation;

      odom.child_frame_id = base_frame_id;
      odom.twist.twist.linear.x = 0.0;
      odom.twist.twist.linear.y = 0.0;
      odom.twist.twist.angular.z = 0.0;

      odom_pub.publish(odom);
    }

    /**
     * @brief publish scan matching status
     */
    void publish_scan_matching_status(const ros::Time &stamp, const std::string &frame_id, pcl::PointCloud<pcl::PointXYZI>::ConstPtr aligned, const std::string &msf_source, const Eigen::Isometry3f &msf_delta)
    {
      if (!status_pub.getNumSubscribers())
      {
        return;
      }

      ScanMatchingStatus status;
      status.header.frame_id = frame_id;
      status.header.stamp = stamp;
      status.has_converged = registration->hasConverged();
      status.matching_error = registration->getFitnessScore();

      const double max_correspondence_dist = 0.5;

      int num_inliers = 0;
      std::vector<int> k_indices;
      std::vector<float> k_sq_dists;
      for (int i = 0; i < aligned->size(); i++)
      {
        const auto &pt = aligned->at(i);
        registration->getSearchMethodTarget()->nearestKSearch(pt, 1, k_indices, k_sq_dists);
        if (k_sq_dists[0] < max_correspondence_dist * max_correspondence_dist)
        {
          num_inliers++;
        }
      }
      status.inlier_fraction = static_cast<float>(num_inliers) / aligned->size();

      status.relative_pose = isometry2pose(Eigen::Isometry3f(registration->getFinalTransformation()).cast<double>());

      if (!msf_source.empty())
      {
        status.prediction_labels.resize(1);
        status.prediction_labels[0].data = msf_source;

        status.prediction_errors.resize(1);
        Eigen::Isometry3f error = Eigen::Isometry3f(registration->getFinalTransformation()).inverse() * msf_delta;
        status.prediction_errors[0] = isometry2pose(error.cast<double>());
      }

      status_pub.publish(status);
    }

  private:
    // ROS topics
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    ros::Subscriber points_sub;
    ros::Subscriber msf_pose_sub;
    ros::Subscriber msf_pose_after_update_sub;

    ros::Publisher odom_pub;
    ros::Publisher trans_pub;
    ros::Publisher aligned_points_pub;
    ros::Publisher status_pub;
    tf::TransformListener tf_listener;
    tf::TransformBroadcaster odom_broadcaster;
    tf::TransformBroadcaster keyframe_broadcaster;

    std::string published_odom_topic;
    std::string points_topic;
    std::string odom_frame_id;
    std::string robot_odom_frame_id;
    ros::Publisher read_until_pub;

    // keyframe parameters
    double keyframe_delta_trans; // minimum distance between keyframes
    double keyframe_delta_angle; //
    double keyframe_delta_time;  //

    // registration validation by thresholding
    bool transform_thresholding; //
    double max_acceptable_trans; //
    double max_acceptable_angle;

    // odometry calculation
    geometry_msgs::PoseWithCovarianceStampedConstPtr msf_pose;
    geometry_msgs::PoseWithCovarianceStampedConstPtr msf_pose_after_update;

    ros::Time prev_time;
    Eigen::Matrix4f prev_trans;                 // previous estimated transform from keyframe
    Eigen::Matrix4f keyframe_pose;              // keyframe pose
    ros::Time keyframe_stamp;                   // keyframe time
    pcl::PointCloud<PointT>::ConstPtr keyframe; // keyframe point cloud

    //
    pcl::Filter<PointT>::Ptr downsample_filter;
    pcl::Registration<PointT, PointT>::Ptr registration;
  };

} // namespace hdl_graph_slam

PLUGINLIB_EXPORT_CLASS(hdl_graph_slam::ScanMatchingOdometryNodelet, nodelet::Nodelet)
