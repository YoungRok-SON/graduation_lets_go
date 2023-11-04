// SPDX-License-Identifier: BSD-2-Clause

#include <string>

#include <ros/ros.h>
#include <ros/time.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace hdl_graph_slam
{

  class PrefilteringNodelet : public nodelet::Nodelet
  {
  public:
    typedef pcl::PointXYZI PointT;

    PrefilteringNodelet() {}
    virtual ~PrefilteringNodelet() {}

    virtual void onInit()
    {
      nh = getNodeHandle();
      private_nh = getPrivateNodeHandle(); // 노들렛은 이렇게 개인 노드 핸들러를 받아줘야 하나봄

      initialize_params(); // 파라미터 로스 파람에서 불러오는 부분

      // 로스 파람에서 빠진거 보면 이건 지금 안쓰는 듯 - 함수도 안쓰는거 많은 듯 함
      if (private_nh.param<bool>("deskewing", false))
      {
        imu_sub = nh.subscribe("/imu/data", 1, &PrefilteringNodelet::imu_callback, this);
      }

      // Subscribe 하는 친구들 이랑 Publish하는 친구들 정의
      points_sub = nh.subscribe("/velodyne_points", 64, &PrefilteringNodelet::cloud_callback, this);
      points_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 32);
      colored_pub = nh.advertise<sensor_msgs::PointCloud2>("/colored_points", 32);
    }

  private:
    /**
     * @brief Downsampling, OutlierPointRemover, Range 기반 필터링 객체 정의
     * @return
     */
    void initialize_params()
    {
      std::string downsample_method = private_nh.param<std::string>("downsample_method", "VOXELGRID");
      double downsample_resolution = private_nh.param<double>("downsample_resolution", 0.1);

      /* Grid 기반 필터링 방법 */
      if (downsample_method == "VOXELGRID")
      {
        std::cout << "downsample: VOXELGRID " << downsample_resolution << std::endl;
        auto voxelgrid = new pcl::VoxelGrid<PointT>();
        voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution); // x, y, z에 대한 복셀 크기
        downsample_filter.reset(voxelgrid);
      }
      else if (downsample_method == "APPROX_VOXELGRID")
      {
        std::cout << "downsample: APPROX_VOXELGRID " << downsample_resolution << std::endl;
        pcl::ApproximateVoxelGrid<PointT>::Ptr approx_voxelgrid(new pcl::ApproximateVoxelGrid<PointT>()); // 이건 왜 Auto로 안받지?
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
      }


      /* Grid 기반 필터링 방법 */
      std::string outlier_removal_method = private_nh.param<std::string>("outlier_removal_method", "STATISTICAL");
      if (outlier_removal_method == "STATISTICAL") // 확률 기반으로 아웃라이어 필터링 조지는 부분
      {
        int mean_k = private_nh.param<int>("statistical_mean_k", 20);
        double stddev_mul_thresh = private_nh.param<double>("statistical_stddev", 1.0);
        std::cout << "outlier_removal: STATISTICAL " << mean_k << " - " << stddev_mul_thresh << std::endl;

        pcl::StatisticalOutlierRemoval<PointT>::Ptr sor(new pcl::StatisticalOutlierRemoval<PointT>());
        sor->setMeanK(mean_k);
        sor->setStddevMulThresh(stddev_mul_thresh);
        outlier_removal_filter = sor;
      }
      else if (outlier_removal_method == "RADIUS") // 주변에 점 있는지 확인하고 일정 개수보다 모자르면 아웃라이어로
      {
        double radius = private_nh.param<double>("radius_radius", 0.8);
        int min_neighbors = private_nh.param<int>("radius_min_neighbors", 2);
        std::cout << "outlier_removal: RADIUS " << radius << " - " << min_neighbors << std::endl;

        pcl::RadiusOutlierRemoval<PointT>::Ptr rad(new pcl::RadiusOutlierRemoval<PointT>());
        rad->setRadiusSearch(radius);
        rad->setMinNeighborsInRadius(min_neighbors);
        outlier_removal_filter = rad;
      }
      else
      {
        std::cout << "outlier_removal: NONE" << std::endl;
      }

      use_distance_filter = private_nh.param<bool>("use_distance_filter", true);

      // 아래 두 개는 그냥 라이다 기준 거리 필터링 하는 것으로 보임
      distance_near_thresh = private_nh.param<double>("distance_near_thresh", 1.0);
      distance_far_thresh = private_nh.param<double>("distance_far_thresh", 100.0);

      // ToDo - 이 링크 나중에 rqt로 어떻게 이뤄졌는지 확인해보기
      base_link_frame = private_nh.param<std::string>("base_link_frame", "");
    }
    
    /**
     * @brief IMU 데이터 들어오면 큐에 저장
     * @param input Sensors_msgs::ImuConstPtr 형식의 IMU 데이터
     * @return Filtered Point Cloud
     */
    void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
    {
      imu_queue.push_back(imu_msg);
    }

    /**
     * @brief Deskewing, Downsampling, OutlierPointRemover, Range 기반 필터링 진행
     * @param Input pcl::PointCloud<PointT> 형식의 PointCloud
     * @return
     */
    void cloud_callback(const pcl::PointCloud<PointT> &src_cloud_r)
    {
      // 이게 로스 PointCloud로 안오고 바로 PCL로 받을 수 있나? 들어오는 bag은 sensor_msgs::PointCloud2일텐데
      pcl::PointCloud<PointT>::ConstPtr src_cloud = src_cloud_r.makeShared();
      
      // Point Cloud 값 없을 경우에는 그냥 리턴
      if (src_cloud->empty())
      {
        return;
      }

      // Deskew도 IMU 데이터 들어오면 해줌. 근데 tf 지정하는 부분은 어디?
      src_cloud = deskewing(src_cloud);

      // if base_link_frame is defined, transform the input cloud to the frame
      if (!base_link_frame.empty())
      {
        if (!tf_listener.canTransform(base_link_frame, src_cloud->header.frame_id, ros::Time(0))) // IMU에 대한 tf 검사도 해야한는 거 아닌가/
        {
          std::cerr << "failed to find transform between " << base_link_frame << " and " << src_cloud->header.frame_id << std::endl;
        }

        tf::StampedTransform transform;
        tf_listener.waitForTransform(base_link_frame, src_cloud->header.frame_id, ros::Time(0), ros::Duration(2.0));
        tf_listener.lookupTransform(base_link_frame, src_cloud->header.frame_id, ros::Time(0), transform);

        pcl::PointCloud<PointT>::Ptr transformed(new pcl::PointCloud<PointT>());
        pcl_ros::transformPointCloud(*src_cloud, *transformed, transform);
        transformed->header.frame_id = base_link_frame;
        transformed->header.stamp = src_cloud->header.stamp;
        src_cloud = transformed;
      }
      // 거리 기반 필터링
      pcl::PointCloud<PointT>::ConstPtr filtered = distance_filter(src_cloud);
      // 다운 샘플링
      filtered = downsample(filtered);
      // 아웃라이어 제거
      filtered = outlier_removal(filtered);
      // 필터링 된 포인트 클라우드 퍼블리쉬
      points_pub.publish(*filtered);
    }

    /**
     * @brief Downsampling 진행, 필터링 객체는 사전에 정의됨
     * @param input 포인트 클라우드
     * @return Downsampled Point Cloud
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
      filtered->header = cloud->header;

      return filtered;
    }
    /**
     * @brief Outlier 삭제 진행, 필터링 객체는 사전에 정의됨
     * @param input 포인트 클라우드
     * @return point CloudRemoved outlier 
     */
    pcl::PointCloud<PointT>::ConstPtr outlier_removal(const pcl::PointCloud<PointT>::ConstPtr &cloud) const
    {
      if (!outlier_removal_filter)
      {
        return cloud;
      }

      pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
      outlier_removal_filter->setInputCloud(cloud);
      outlier_removal_filter->filter(*filtered);
      filtered->header = cloud->header;

      return filtered;
    }
    /**
     * @brief 거리를 기반으로 지정된 거리 안에 있는 포인트만 사용
     * @param input 포인트 클라우드
     * @return Filtered Point Cloud
     */
    pcl::PointCloud<PointT>::ConstPtr distance_filter(const pcl::PointCloud<PointT>::ConstPtr &cloud) const
    {
      pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
      filtered->reserve(cloud->size());

      // std::back_inserter는 삽입 반복자를 생성하는 함수로, 새로운 요소들이 filtered->points의 끝에 추가되도록 합니다.
      std::copy_if(cloud->begin(), cloud->end(), std::back_inserter(filtered->points), [&](const PointT &p)
                   { // 이는 람다 함수의 선언 부분입니다. &는 람다가 외부 범위의 모든 변수에 접근할 수 있음을 나타냅니다. 람다 함수는 p라는 이름의 포인트를 매개변수로 받습니다.
      double d = p.getVector3fMap().norm(); // 포인트 p의 좌표를 3D 벡터로 가져온 후, 그 벡터의 크기(또는 norm)을 계산합니다. 이것은 원점에서 포인트까지의 거리를 나타냅니다.
      return d > distance_near_thresh && d < distance_far_thresh; });

      filtered->width = filtered->size();
      filtered->height = 1;
      filtered->is_dense = false;

      filtered->header = cloud->header;

      return filtered;
    }

    /**
     * @brief IMU 데이터를 사용해서 Deskewing 진행
     * @param input 포인트 클라우드
     * @return Deskewed Point Cloud
     */
    pcl::PointCloud<PointT>::ConstPtr deskewing(const pcl::PointCloud<PointT>::ConstPtr &cloud)
    {
      ros::Time stamp = pcl_conversions::fromPCL(cloud->header.stamp);
      if (imu_queue.empty())
      {
        return cloud;
      }

      // the color encodes the point number in the point sequence
      if (colored_pub.getNumSubscribers()) // 누군가 colored_pub을 받고 있을 경우에만 아래 내용을 진행한다고
      {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>());
        colored->header = cloud->header;
        colored->is_dense = cloud->is_dense;
        colored->width = cloud->width;
        colored->height = cloud->height;
        colored->resize(cloud->size());

        for (int i = 0; i < cloud->size(); i++)
        {
          double t = static_cast<double>(i) / cloud->size();
          colored->at(i).getVector4fMap() = cloud->at(i).getVector4fMap();
          colored->at(i).r = 255 * t;
          colored->at(i).g = 128;
          colored->at(i).b = 255 * (1 - t);
        }
        colored_pub.publish(*colored);
      }

      sensor_msgs::ImuConstPtr imu_msg = imu_queue.front();

      auto loc = imu_queue.begin();
      for (; loc != imu_queue.end(); loc++)
      {
        imu_msg = (*loc); // Stamp 비교해서 마지막 데이터의 포인터만 가지고 오는 형식
        if ((*loc)->header.stamp > stamp)  
        {
          break;
        }
      }

      imu_queue.erase(imu_queue.begin(), loc); // begine부터 loc까지 해당되는 부분을 삭제

      Eigen::Vector3f ang_v(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z); // 마지막 데이터의 각속도 정보를 가져옴
      ang_v *= -1; // 이건 모지

      pcl::PointCloud<PointT>::Ptr deskewed(new pcl::PointCloud<PointT>());
      deskewed->header = cloud->header;
      deskewed->is_dense = cloud->is_dense;
      deskewed->width = cloud->width;
      deskewed->height = cloud->height;
      deskewed->resize(cloud->size());

      double scan_period = private_nh.param<double>("scan_period", 0.1);
      for (int i = 0; i < cloud->size(); i++)
      {
        const auto &pt = cloud->at(i);

        // TODO: transform IMU data into the LIDAR frame
        double delta_t = scan_period * static_cast<double>(i) / cloud->size();
        Eigen::Quaternionf delta_q(1, delta_t / 2.0 * ang_v[0], delta_t / 2.0 * ang_v[1], delta_t / 2.0 * ang_v[2]);
        Eigen::Vector3f pt_ = delta_q.inverse() * pt.getVector3fMap();

        deskewed->at(i) = cloud->at(i); // 시간에 따라 밀린만큼 점들을 움직여서 다시 저장
        deskewed->at(i).getVector3fMap() = pt_;
      }

      return deskewed;
    }

  private:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    ros::Subscriber imu_sub;
    std::vector<sensor_msgs::ImuConstPtr> imu_queue;

    ros::Subscriber points_sub;
    ros::Publisher points_pub;

    ros::Publisher colored_pub;

    tf::TransformListener tf_listener;

    std::string base_link_frame;

    bool use_distance_filter;
    double distance_near_thresh;
    double distance_far_thresh;

    pcl::Filter<PointT>::Ptr downsample_filter;
    pcl::Filter<PointT>::Ptr outlier_removal_filter;
  };

} // namespace hdl_graph_slam

PLUGINLIB_EXPORT_CLASS(hdl_graph_slam::PrefilteringNodelet, nodelet::Nodelet)
