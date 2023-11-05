// SPDX-License-Identifier: BSD-2-Clause

#include <ctime>
#include <mutex>
#include <atomic>
#include <memory>
#include <iomanip>
#include <iostream>
#include <unordered_map>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>

#include <ros/ros.h>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <pcl_ros/point_cloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <std_msgs/Time.h>
// #include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <nmea_msgs/Sentence.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <hdl_graph_slam/FloorCoeffs.h>

#include <hdl_graph_slam/SaveMap.h>
#include <hdl_graph_slam/LoadGraph.h>
#include <hdl_graph_slam/DumpGraph.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <hdl_graph_slam/ros_utils.hpp>
#include <hdl_graph_slam/ros_time_hash.hpp>

#include <hdl_graph_slam/graph_slam.hpp>
#include <hdl_graph_slam/keyframe.hpp>
#include <hdl_graph_slam/keyframe_updater.hpp>
#include <hdl_graph_slam/loop_detector.hpp>
#include <hdl_graph_slam/information_matrix_calculator.hpp>
#include <hdl_graph_slam/map_cloud_generator.hpp>
#include <hdl_graph_slam/nmea_sentence_parser.hpp>

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <g2o/edge_se3_priorvec.hpp>
#include <g2o/edge_se3_priorquat.hpp>

namespace hdl_graph_slam
{

  class HdlGraphSlamNodelet : public nodelet::Nodelet
  {
  public:
    typedef pcl::PointXYZI PointT;
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::PointCloud2> ApproxSyncPolicy;

    HdlGraphSlamNodelet() {}
    virtual ~HdlGraphSlamNodelet() {}

    virtual void onInit()
    {
      nh = getNodeHandle();
      mt_nh = getMTNodeHandle();
      private_nh = getPrivateNodeHandle();

      // init parameters
      published_odom_topic = private_nh.param<std::string>("published_odom_topic", "/odom");
      map_frame_id = private_nh.param<std::string>("map_frame_id", "map");
      odom_frame_id = private_nh.param<std::string>("odom_frame_id", "odom");
      map_cloud_resolution = private_nh.param<double>("map_cloud_resolution", 0.05);
      trans_odom2map.setIdentity();

      // 한 업데이트(최적화)마다 최대로 최적화할 수 있는 키 프레임
      // Q. 그럼 1번부터 10번까지 했으면 10번부터 19번까지 진행되는건가?
      max_keyframes_per_update = private_nh.param<int>("max_keyframes_per_update", 10);

      // node, edge 및 graph 관련 객체 초기화
      anchor_node = nullptr;
      anchor_edge = nullptr;
      floor_plane_node = nullptr;
      graph_slam.reset(new GraphSLAM(private_nh.param<std::string>("g2o_solver_type", "lm_var")));

      // 이 친구들은 뭐하는 친구들인지 좀 더 봐야할 듯
      keyframe_updater.reset(new KeyframeUpdater(private_nh));
      loop_detector.reset(new LoopDetector(private_nh)); // Loop Closure 클래스 객체 생성
      map_cloud_generator.reset(new MapCloudGenerator());
      inf_calclator.reset(new InformationMatrixCalculator(private_nh));
      nmea_parser.reset(new NmeaSentenceParser());

      // 추가 엣지를 위한 정보들
      gps_time_offset = private_nh.param<double>("gps_time_offset", 0.0);
      gps_edge_stddev_xy = private_nh.param<double>("gps_edge_stddev_xy", 10000.0);
      gps_edge_stddev_z = private_nh.param<double>("gps_edge_stddev_z", 10.0);
      floor_edge_stddev = private_nh.param<double>("floor_edge_stddev", 10.0);

      // 추가 엣지를 위한 정보들
      imu_time_offset = private_nh.param<double>("imu_time_offset", 0.0);
      enable_imu_orientation = private_nh.param<bool>("enable_imu_orientation", false);
      enable_imu_acceleration = private_nh.param<bool>("enable_imu_acceleration", false);
      imu_orientation_edge_stddev = private_nh.param<double>("imu_orientation_edge_stddev", 0.1);
      imu_acceleration_edge_stddev = private_nh.param<double>("imu_acceleration_edge_stddev", 3.0);

      points_topic = private_nh.param<std::string>("points_topic", "/velodyne_points");

      // subscribers
      odom_sub.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(mt_nh, published_odom_topic, 256));
      cloud_sub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(mt_nh, "/filtered_points", 32));
      sync.reset(new message_filters::Synchronizer<ApproxSyncPolicy>(ApproxSyncPolicy(32), *odom_sub, *cloud_sub)); // 포인트클라우드랑 포즈 정보랑 엮으려고 필터 사용
      sync->registerCallback(boost::bind(&HdlGraphSlamNodelet::cloud_callback, this, _1, _2));
      imu_sub = nh.subscribe("/gpsimu_driver/imu_data", 1024, &HdlGraphSlamNodelet::imu_callback, this);
      floor_sub = nh.subscribe("/floor_detection/floor_coeffs", 1024, &HdlGraphSlamNodelet::floor_coeffs_callback, this);

      // Debug Publisher
      debug_loop_closer_target_pub = nh.advertise<sensor_msgs::PointCloud2>("/hdl_graph_slam/debug/loop_closer_target", 1, true);
      debug_loop_closer_source_pub = nh.advertise<sensor_msgs::PointCloud2>("/hdl_graph_slam/debug/loop_closer_source", 1, true);
      debug_loop_closure_target_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/hdl_graph_slam/debug/loop_closure_target_pose", 1, true);
      debug_loop_closure_source_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/hdl_graph_slam/debug/loop_closure_source_pose", 1, true);

      if (private_nh.param<bool>("enable_gps", true))
      {
        gps_sub = mt_nh.subscribe("/gps/geopoint", 1024, &HdlGraphSlamNodelet::gps_callback, this);
        nmea_sub = mt_nh.subscribe("/gpsimu_driver/nmea_sentence", 1024, &HdlGraphSlamNodelet::nmea_callback, this);
        navsat_sub = mt_nh.subscribe("/gps/navsat", 1024, &HdlGraphSlamNodelet::navsat_callback, this);
      }

      // publishers
      markers_pub = mt_nh.advertise<visualization_msgs::MarkerArray>("/hdl_graph_slam/markers", 16);
      odom2map_pub = mt_nh.advertise<geometry_msgs::TransformStamped>("/hdl_graph_slam/odom2pub", 16);
      map_points_pub = mt_nh.advertise<sensor_msgs::PointCloud2>("/hdl_graph_slam/map_points", 1, true);
      read_until_pub = mt_nh.advertise<std_msgs::Header>("/hdl_graph_slam/read_until", 32);

      load_service_server = mt_nh.advertiseService("/hdl_graph_slam/load", &HdlGraphSlamNodelet::load_service, this);
      dump_service_server = mt_nh.advertiseService("/hdl_graph_slam/dump", &HdlGraphSlamNodelet::dump_service, this);
      save_map_service_server = mt_nh.advertiseService("/hdl_graph_slam/save_map", &HdlGraphSlamNodelet::save_map_service, this);

      graph_updated = false;
      double graph_update_interval = private_nh.param<double>("graph_update_interval", 3.0);
      double map_cloud_update_interval = private_nh.param<double>("map_cloud_update_interval", 10.0);
      optimization_timer = mt_nh.createWallTimer(ros::WallDuration(graph_update_interval), &HdlGraphSlamNodelet::optimization_timer_callback, this); //3초마다 한번씩 퍼블리쉬
      map_publish_timer = mt_nh.createWallTimer(ros::WallDuration(map_cloud_update_interval), &HdlGraphSlamNodelet::map_points_publish_timer_callback, this);
    }

  private:
    /**
     * @brief received point clouds are pushed to #keyframe_queue
     * @param odom_msg
     * @param cloud_msg
     */
    void cloud_callback(const geometry_msgs::PoseStampedConstPtr &odom_msg, const sensor_msgs::PointCloud2::ConstPtr &cloud_msg) // odometry 데이터랑 포인트 클라우드를 같이 수신
    {
      const ros::Time &stamp = cloud_msg->header.stamp;
      Eigen::Isometry3d odom = poseStampedPtr2isometry(odom_msg);

      pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
      pcl::fromROSMsg(*cloud_msg, *cloud);
      if (base_frame_id.empty())
      {
        base_frame_id = cloud_msg->header.frame_id;
      }

      if (!keyframe_updater->update(odom)) // 이전 keyframe과의 거리 계산을 통해 짧으면 false, 길면 true
      {
        std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
        if (keyframe_queue.empty()) // keyframe_queue가 비어있다면
        { // 이 read_untill 이라는 부분 뭔지 잘 모르겠음
          std_msgs::Header read_until;
          read_until.stamp = stamp + ros::Duration(10, 0);
          read_until.frame_id = points_topic;
          read_until_pub.publish(read_until);
          read_until.frame_id = "/filtered_points";
          read_until_pub.publish(read_until);
        }

        return;
      }

      double accum_d = keyframe_updater->get_accum_distance();
      KeyFrame::Ptr keyframe(new KeyFrame(stamp, odom, accum_d, cloud)); // Keyframe 구조체에 대이터를 넣은 다음에

      std::lock_guard<std::mutex> lock(keyframe_queue_mutex); // 뮤텍스 걸고
      keyframe_queue.push_back(keyframe); // 생성한 키프레임 객체를 큐에 넣기
    }

    /**
     * @brief this method adds all the keyframes in #keyframe_queue to the pose graph (odometry edges)
     * @return if true, at least one keyframe was added to the pose graph
     */
    bool flush_keyframe_queue()
    {
      std::lock_guard<std::mutex> lock(keyframe_queue_mutex);

      // 키프레임큐가 비어있다면 그냥 리턴
      if (keyframe_queue.empty())
      {
        return false;
      }

      trans_odom2map_mutex.lock();
      Eigen::Isometry3d odom2map(trans_odom2map.cast<double>()); // trans_odom2map 변수 값을 double로 캐스팅해서 odom2map이라는 변수 생성
      // trans_odom2map은 어디서 가져오는거? -> delta transformatoin
      trans_odom2map_mutex.unlock();

      std::cout << "flush_keyframe_queue - keyframes len:" << keyframes.size() << std::endl; //Keyframe queue도 있는데 이건 왜 따로 벡터로 지정해놨지?
      int num_processed = 0;
      for (int i = 0; i < std::min<int>(keyframe_queue.size(), max_keyframes_per_update); i++) // Keyframe queue에 있는 개수가 min보다 작으면 개수만큼만 진행
      {
        num_processed = i;

        const auto &keyframe = keyframe_queue[i];
        // new_keyframes will be tested later for loop closure
        new_keyframes.push_back(keyframe);

        // add pose node
        Eigen::Isometry3d odom = odom2map * keyframe->odom;
        keyframe->node = graph_slam->add_se3_node(odom); // Keyframe 객체를 생성할 때 노드에 대한 값이 비어 있음. 그걸 이때 채워넣는듯 그리고 그래프 안에도 넣어줌
        keyframe_hash[keyframe->stamp] = keyframe; // floor_queue에서 매칭되는 키프레임을 찾을 때 해쉬값을 사용해서 찾음

        // fix the first node
        if (keyframes.empty() && new_keyframes.size() == 1)
        {

          if (private_nh.param<bool>("fix_first_node", false))
          {
            Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
            std::stringstream sst(private_nh.param<std::string>("fix_first_node_stddev", "1 1 1 1 1 1"));
            for (int i = 0; i < 6; i++)
            {
              double stddev = 1.0;
              sst >> stddev;
              inf(i, i) = 1.0 / stddev;
            }

            anchor_node = graph_slam->add_se3_node(Eigen::Isometry3d::Identity()); // 처음 노드
            anchor_node->setFixed(true);  // 고정
            anchor_edge = graph_slam->add_se3_edge(anchor_node, keyframe->node, Eigen::Isometry3d::Identity(), inf); // 앵커노드와 첫번째 키프레임 사이 에지를 지정
          }
        }

        if (i == 0 && keyframes.empty()) // 만약 Keyframe들을 모아놓는 벡터에 아무것도 없으면 그냥 넘어감
        {
          continue; // 이 std::vector<KeyFrame::Ptr> 형태의 변수 용도가 뭔지? 
        }

        // add edge between consecutive keyframes
        const auto &prev_keyframe = i == 0 ? keyframes.back() : keyframe_queue[i - 1];
        // keyframe_queue는 어디선가 계속 초기화가 되나?
        // 일단 i==0 일때 현재 키프레임과 이전 키프레임을 엮어줘야하니 prev_keyframe을 지정해주는 것으로 보임

        Eigen::Isometry3d relative_pose = keyframe->odom.inverse() * prev_keyframe->odom; // 현재와 이전 사이의 상대 포즈 구하고
        Eigen::MatrixXd information = inf_calclator->calc_information_matrix(keyframe->cloud, prev_keyframe->cloud, relative_pose);
        // 뭐 어떻게 해서 information matrix까지 구함 -> 나중에 읽어볼거
        auto edge = graph_slam->add_se3_edge(keyframe->node, prev_keyframe->node, relative_pose, information);
        graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("odometry_edge_robust_kernel", "NONE"), private_nh.param<double>("odometry_edge_robust_kernel_size", 1.0));
        // 이건 또 뭘까
      }

      std_msgs::Header read_until;
      read_until.stamp = keyframe_queue[num_processed]->stamp + ros::Duration(10, 0);
      read_until.frame_id = points_topic;
      read_until_pub.publish(read_until);
      read_until.frame_id = "/filtered_points";
      read_until_pub.publish(read_until);


      // 아 3초에 한번씩 도니까 한번 싹 밀어 넣은 다음에 처리한건 다 지우는구나
      keyframe_queue.erase(keyframe_queue.begin(), keyframe_queue.begin() + num_processed + 1);
      return true;
    }

    void nmea_callback(const nmea_msgs::SentenceConstPtr &nmea_msg)
    {
      GPRMC grmc = nmea_parser->parse(nmea_msg->sentence);

      if (grmc.status != 'A')
      {
        return;
      }

      geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
      gps_msg->header = nmea_msg->header;
      gps_msg->position.latitude = grmc.latitude;
      gps_msg->position.longitude = grmc.longitude;
      gps_msg->position.altitude = NAN;

      gps_callback(gps_msg);
    }

    void navsat_callback(const sensor_msgs::NavSatFixConstPtr &navsat_msg)
    {
      geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
      gps_msg->header = navsat_msg->header;
      gps_msg->position.latitude = navsat_msg->latitude;
      gps_msg->position.longitude = navsat_msg->longitude;
      gps_msg->position.altitude = navsat_msg->altitude;
      gps_callback(gps_msg);
    }

    /**
     * @brief received gps data is added to #gps_queue
     * @param gps_msg
     */
    void gps_callback(const geographic_msgs::GeoPointStampedPtr &gps_msg)
    {
      std::lock_guard<std::mutex> lock(gps_queue_mutex);
      gps_msg->header.stamp += ros::Duration(gps_time_offset);
      gps_queue.push_back(gps_msg);
    }

    /**
     * @brief
     * @return
     */
    bool flush_gps_queue()
    {
      std::lock_guard<std::mutex> lock(gps_queue_mutex);

      if (keyframes.empty() || gps_queue.empty())
      {
        return false;
      }

      bool updated = false;
      auto gps_cursor = gps_queue.begin();

      for (auto &keyframe : keyframes)
      {
        if (keyframe->stamp > gps_queue.back()->header.stamp)
        {
          break;
        }

        if (keyframe->stamp < (*gps_cursor)->header.stamp || keyframe->utm_coord)
        {
          continue;
        }

        // find the gps data which is closest to the keyframe
        auto closest_gps = gps_cursor;
        for (auto gps = gps_cursor; gps != gps_queue.end(); gps++)
        {
          auto dt = ((*closest_gps)->header.stamp - keyframe->stamp).toSec();
          auto dt2 = ((*gps)->header.stamp - keyframe->stamp).toSec();
          if (std::abs(dt) < std::abs(dt2))
          {
            break;
          }

          closest_gps = gps;
        }

        // if the time residual between the gps and keyframe is too large, skip it
        gps_cursor = closest_gps;
        if (0.2 < std::abs(((*closest_gps)->header.stamp - keyframe->stamp).toSec()))
        {
          continue;
        }

        // convert (latitude, longitude, altitude) -> (easting, northing, altitude) in UTM coordinate
        geodesy::UTMPoint utm;
        geodesy::fromMsg((*closest_gps)->position, utm);
        Eigen::Vector3d xyz(utm.easting, utm.northing, utm.altitude);

        // the first gps data position will be the origin of the map
        if (!zero_utm)
        {
          zero_utm = xyz;
        }
        xyz -= (*zero_utm);

        keyframe->utm_coord = xyz;

        g2o::OptimizableGraph::Edge *edge;
        if (std::isnan(xyz.z()))
        {
          Eigen::Matrix2d information_matrix = Eigen::Matrix2d::Identity() / gps_edge_stddev_xy;
          edge = graph_slam->add_se3_prior_xy_edge(keyframe->node, xyz.head<2>(), information_matrix);
        }
        else
        {
          Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Identity();
          information_matrix.block<2, 2>(0, 0) /= gps_edge_stddev_xy;
          information_matrix(2, 2) /= gps_edge_stddev_z;
          edge = graph_slam->add_se3_prior_xyz_edge(keyframe->node, xyz, information_matrix);
        }
        graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("gps_edge_robust_kernel", "NONE"), private_nh.param<double>("gps_edge_robust_kernel_size", 1.0));

        updated = true;
      }

      auto remove_loc = std::upper_bound(gps_queue.begin(), gps_queue.end(), keyframes.back()->stamp, [=](const ros::Time &stamp, const geographic_msgs::GeoPointStampedConstPtr &geopoint)
                                         { return stamp < geopoint->header.stamp; });
      gps_queue.erase(gps_queue.begin(), remove_loc);
      return updated;
    }

    void imu_callback(const sensor_msgs::ImuPtr &imu_msg)
    {
      if (!enable_imu_orientation && !enable_imu_acceleration)
      {
        return;
      }

      std::lock_guard<std::mutex> lock(imu_queue_mutex);
      imu_msg->header.stamp += ros::Duration(imu_time_offset);
      imu_queue.push_back(imu_msg);
    }

    bool flush_imu_queue()
    {
      std::lock_guard<std::mutex> lock(imu_queue_mutex);
      if (keyframes.empty() || imu_queue.empty() || base_frame_id.empty())
      {
        return false;
      }

      bool updated = false;
      auto imu_cursor = imu_queue.begin();

      for (auto &keyframe : keyframes)
      {
        if (keyframe->stamp > imu_queue.back()->header.stamp)
        {
          break;
        }

        if (keyframe->stamp < (*imu_cursor)->header.stamp || keyframe->acceleration)
        {
          continue;
        }

        // find imu data which is closest to the keyframe
        auto closest_imu = imu_cursor;
        for (auto imu = imu_cursor; imu != imu_queue.end(); imu++)
        {
          auto dt = ((*closest_imu)->header.stamp - keyframe->stamp).toSec();
          auto dt2 = ((*imu)->header.stamp - keyframe->stamp).toSec();
          if (std::abs(dt) < std::abs(dt2))
          {
            break;
          }

          closest_imu = imu;
        }

        imu_cursor = closest_imu;
        if (0.2 < std::abs(((*closest_imu)->header.stamp - keyframe->stamp).toSec()))
        {
          continue;
        }

        const auto &imu_ori = (*closest_imu)->orientation;
        const auto &imu_acc = (*closest_imu)->linear_acceleration;

        geometry_msgs::Vector3Stamped acc_imu;
        geometry_msgs::Vector3Stamped acc_base;
        geometry_msgs::QuaternionStamped quat_imu;
        geometry_msgs::QuaternionStamped quat_base;

        quat_imu.header.frame_id = acc_imu.header.frame_id = (*closest_imu)->header.frame_id;
        quat_imu.header.stamp = acc_imu.header.stamp = ros::Time(0);
        acc_imu.vector = (*closest_imu)->linear_acceleration;
        quat_imu.quaternion = (*closest_imu)->orientation;

        try
        {
          tf_listener.transformVector(base_frame_id, acc_imu, acc_base);
          tf_listener.transformQuaternion(base_frame_id, quat_imu, quat_base);
        }
        catch (std::exception &e)
        {
          std::cerr << "failed to find transform!!" << std::endl;
          return false;
        }

        keyframe->acceleration = Eigen::Vector3d(acc_base.vector.x, acc_base.vector.y, acc_base.vector.z);
        keyframe->orientation = Eigen::Quaterniond(quat_base.quaternion.w, quat_base.quaternion.x, quat_base.quaternion.y, quat_base.quaternion.z);
        keyframe->orientation = keyframe->orientation;
        if (keyframe->orientation->w() < 0.0)
        {
          keyframe->orientation->coeffs() = -keyframe->orientation->coeffs();
        }

        if (enable_imu_orientation)
        {
          Eigen::MatrixXd info = Eigen::MatrixXd::Identity(3, 3) / imu_orientation_edge_stddev;
          auto edge = graph_slam->add_se3_prior_quat_edge(keyframe->node, *keyframe->orientation, info);
          graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("imu_orientation_edge_robust_kernel", "NONE"), private_nh.param<double>("imu_orientation_edge_robust_kernel_size", 1.0));
        }

        if (enable_imu_acceleration)
        {
          Eigen::MatrixXd info = Eigen::MatrixXd::Identity(3, 3) / imu_acceleration_edge_stddev;
          g2o::OptimizableGraph::Edge *edge = graph_slam->add_se3_prior_vec_edge(keyframe->node, -Eigen::Vector3d::UnitZ(), *keyframe->acceleration, info);
          graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("imu_acceleration_edge_robust_kernel", "NONE"), private_nh.param<double>("imu_acceleration_edge_robust_kernel_size", 1.0));
        }
        updated = true;
      }

      auto remove_loc = std::upper_bound(imu_queue.begin(), imu_queue.end(), keyframes.back()->stamp, [=](const ros::Time &stamp, const sensor_msgs::ImuConstPtr &imu)
                                         { return stamp < imu->header.stamp; });
      imu_queue.erase(imu_queue.begin(), remove_loc);

      return updated;
    }

    /**
     * @brief received floor coefficients are added to #floor_coeffs_queue
     * @param floor_coeffs_msg
     */
    void floor_coeffs_callback(const hdl_graph_slam::FloorCoeffsConstPtr &floor_coeffs_msg)
    {
      if (floor_coeffs_msg->coeffs.empty())
      {
        return;
      }

      std::lock_guard<std::mutex> lock(floor_coeffs_queue_mutex);
      floor_coeffs_queue.push_back(floor_coeffs_msg);
    }

    /**
     * @brief this methods associates floor coefficients messages with registered keyframes, and then adds the associated coeffs to the pose graph
     * @return if true, at least one floor plane edge is added to the pose graph
     */
    bool flush_floor_queue()
    {
      std::lock_guard<std::mutex> lock(floor_coeffs_queue_mutex);

      if (keyframes.empty())
      {
        return false;
      }

      const auto &latest_keyframe_stamp = keyframes.back()->stamp;

      bool updated = false;
      for (const auto &floor_coeffs : floor_coeffs_queue)
      {
        if (floor_coeffs->header.stamp > latest_keyframe_stamp)
        {
          break;
        }

        auto found = keyframe_hash.find(floor_coeffs->header.stamp);
        if (found == keyframe_hash.end())
        {
          continue;
        }

        if (!floor_plane_node)
        {
          floor_plane_node = graph_slam->add_plane_node(Eigen::Vector4d(0.0, 0.0, 1.0, 0.0));
          floor_plane_node->setFixed(true);
        }

        const auto &keyframe = found->second;

        Eigen::Vector4d coeffs(floor_coeffs->coeffs[0], floor_coeffs->coeffs[1], floor_coeffs->coeffs[2], floor_coeffs->coeffs[3]);
        Eigen::Matrix3d information = Eigen::Matrix3d::Identity() * (1.0 / floor_edge_stddev);
        auto edge = graph_slam->add_se3_plane_edge(keyframe->node, floor_plane_node, coeffs, information);
        graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("floor_edge_robust_kernel", "NONE"), private_nh.param<double>("floor_edge_robust_kernel_size", 1.0));

        keyframe->floor_coeffs = coeffs;

        updated = true;
      }

      auto remove_loc = std::upper_bound(floor_coeffs_queue.begin(), floor_coeffs_queue.end(), latest_keyframe_stamp, [=](const ros::Time &stamp, const hdl_graph_slam::FloorCoeffsConstPtr &coeffs)
                                         { return stamp < coeffs->header.stamp; });
      floor_coeffs_queue.erase(floor_coeffs_queue.begin(), remove_loc);

      return updated;
    }

    /**
     * @brief generate map point cloud and publish it
     * @param event
     */
    void map_points_publish_timer_callback(const ros::WallTimerEvent &event)
    {
      if (!map_points_pub.getNumSubscribers() || !graph_updated)
      {
        return;
      }

      std::vector<KeyFrameSnapshot::Ptr> snapshot;

      keyframes_snapshot_mutex.lock();
      snapshot = keyframes_snapshot;
      keyframes_snapshot_mutex.unlock();

      auto cloud = map_cloud_generator->generate(snapshot, map_cloud_resolution);
      if (!cloud)
      {
        return;
      }

      cloud->header.frame_id = map_frame_id;
      cloud->header.stamp = snapshot.back()->cloud->header.stamp;

      sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
      pcl::toROSMsg(*cloud, *cloud_msg);

      map_points_pub.publish(cloud_msg);
    }

    /**
     * @brief this methods adds all the data in the queues to the pose graph, and then optimizes the pose graph
     * @param event
     */
    void optimization_timer_callback(const ros::WallTimerEvent &event)
    {
      std::lock_guard<std::mutex> lock(main_thread_mutex);

      // add keyframes and floor coeffs in the queues to the pose graph
      bool keyframe_updated = flush_keyframe_queue();

      if (!keyframe_updated)
      {
        std_msgs::Header read_until;
        read_until.stamp = ros::Time::now() + ros::Duration(30, 0);
        read_until.frame_id = points_topic;
        read_until_pub.publish(read_until);
        read_until.frame_id = "/filtered_points";
        read_until_pub.publish(read_until);
      }


      // 어떠한 키프레임과 큐도 업데이트가 되지 않은 경우 그냥 return
      if (!keyframe_updated & !flush_floor_queue() & !flush_gps_queue() & !flush_imu_queue())
      {
        return;
      }

      // loop detection
      std::vector<Loop::Ptr> loops = loop_detector->detect(keyframes, new_keyframes, *graph_slam);
      // 새로운 키프래임과 이전 키프레임 사이의 매칭 진행? -> N(기존 키프레임들) : M(새로운 키프레임들) 사이의 매칭을 진행해서 여러개가 나올 수 있음.
      for (const auto &loop : loops)
      {
        Eigen::Isometry3d relpose(loop->relative_pose.cast<double>());
        Eigen::MatrixXd information_matrix = inf_calclator->calc_information_matrix(loop->key1->cloud, loop->key2->cloud, relpose);
        auto edge = graph_slam->add_se3_edge(loop->key1->node, loop->key2->node, relpose, information_matrix);
        graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("loop_closure_edge_robust_kernel", "NONE"), private_nh.param<double>("loop_closure_edge_robust_kernel_size", 1.0));
        
        // Debug Publisher
        debug_loop_closure_keyframe_mutex.lock();
        if(debug_loop_closer_target_pub.getNumSubscribers() && debug_loop_closer_source_pub.getNumSubscribers())
        {
          debug_loop_closure_points_pose(loop);
        }
        debug_loop_closure_keyframe_mutex.unlock();
      }

      std::copy(new_keyframes.begin(), new_keyframes.end(), std::back_inserter(keyframes));
      // new_keyframes의 모든 원소들이 keyframes의 끝에 추가
      new_keyframes.clear();

      // move the first node anchor position to the current estimate of the first node pose
      // so the first node moves freely while trying to stay around the origin
      if (anchor_node && private_nh.param<bool>("fix_first_node_adaptive", true))
      {
        Eigen::Isometry3d anchor_target = static_cast<g2o::VertexSE3 *>(anchor_edge->vertices()[1])->estimate();
        anchor_node->setEstimate(anchor_target);
      }

      // optimize the pose graph
      int num_iterations = private_nh.param<int>("g2o_solver_num_iterations", 1024);
      graph_slam->optimize(num_iterations); // 지정한 회수만큼 최적화 진행

      // publish tf
      const auto &keyframe = keyframes.back();
      Eigen::Isometry3d trans = keyframe->node->estimate() * keyframe->odom.inverse();
      trans_odom2map_mutex.lock();
      trans_odom2map = trans.matrix().cast<float>(); // 이 값은 왜 이렇게 해놓지?
      trans_odom2map_mutex.unlock();

      // 벡터인 Keyframes에 있는 객체들을 복사해서 snapshot이라는 벡터 컨테이너에 KeyFrameSnapshot이라는 형태로 저장
      // 결국 새로운 키프래임들을 같이 하나의 맵 데이터로 저장
      std::vector<KeyFrameSnapshot::Ptr> snapshot(keyframes.size());
      std::transform(keyframes.begin(), keyframes.end(), snapshot.begin(), [=](const KeyFrame::Ptr &k)
                     { return std::make_shared<KeyFrameSnapshot>(k); });

      keyframes_snapshot_mutex.lock();
      keyframes_snapshot.swap(snapshot);
      keyframes_snapshot_mutex.unlock();
      graph_updated = true;

      if (odom2map_pub.getNumSubscribers())
      {
        geometry_msgs::TransformStamped ts = matrix2transform(keyframe->stamp, trans.matrix().cast<float>(), map_frame_id, odom_frame_id);
        odom2map_pub.publish(ts);
      }

      if (markers_pub.getNumSubscribers())
      {
        auto markers = create_marker_array(ros::Time::now());
        markers_pub.publish(markers);
      }
    }
    /**
     * @brief Publish loop closre pose with point cloud
     * @param loop - loop closure data
     */
    void debug_loop_closure_points_pose(Loop::Ptr loop)
    {
      // Target Pose
      geometry_msgs::PoseStamped pose_msg_target;
      pose_msg_target.header.frame_id = map_frame_id;
      pose_msg_target.header.stamp = ros::Time::now();
      pose_msg_target.pose.position.x = loop->key1->node->estimate().translation().x();
      pose_msg_target.pose.position.y = loop->key1->node->estimate().translation().y();
      pose_msg_target.pose.position.z = loop->key1->node->estimate().translation().z();
      Eigen::Quaterniond quat_target(loop->key1->node->estimate().rotation());
      pose_msg_target.pose.orientation.x = quat_target.x();
      pose_msg_target.pose.orientation.y = quat_target.y();
      pose_msg_target.pose.orientation.z = quat_target.z();
      pose_msg_target.pose.orientation.w = quat_target.w();
      debug_loop_closure_target_pose_pub.publish(pose_msg_target);
      // Source Pose
      geometry_msgs::PoseStamped pose_msg_source;
      pose_msg_source.header.frame_id = map_frame_id;
      pose_msg_source.header.stamp = ros::Time::now();
      pose_msg_source.pose.position.x = loop->key2->node->estimate().translation().x();
      pose_msg_source.pose.position.y = loop->key2->node->estimate().translation().y();
      pose_msg_source.pose.position.z = loop->key2->node->estimate().translation().z();
      Eigen::Quaterniond quat_source(loop->key2->node->estimate().rotation());
      pose_msg_source.pose.orientation.x = quat_source.x();
      pose_msg_source.pose.orientation.y = quat_source.y();
      pose_msg_source.pose.orientation.z = quat_source.z();
      pose_msg_source.pose.orientation.w = quat_source.w();
      debug_loop_closure_source_pose_pub.publish(pose_msg_source);
      
      geometry_msgs::TransformStamped transformStamped_target;

      transformStamped_target.header.frame_id = "map";
      transformStamped_target.child_frame_id = "debug_loop_closure_target_pose";
      transformStamped_target.transform.translation.x = loop->key1->node->estimate().translation().x();
      transformStamped_target.transform.translation.y = loop->key1->node->estimate().translation().y();
      transformStamped_target.transform.translation.z = loop->key1->node->estimate().translation().z();
      transformStamped_target.transform.rotation.x    = quat_target.x();
      transformStamped_target.transform.rotation.y    = quat_target.y();
      transformStamped_target.transform.rotation.z    = quat_target.z();
      transformStamped_target.transform.rotation.w    = quat_target.w();
      transformStamped_target.header.stamp = ros::Time::now();
      
      geometry_msgs::TransformStamped transformStamped_source;
      transformStamped_source.header.frame_id = "map";
      transformStamped_source.child_frame_id = "debug_loop_closure_source_pose";
      transformStamped_source.transform.translation.x = loop->key2->node->estimate().translation().x();
      transformStamped_source.transform.translation.y = loop->key2->node->estimate().translation().y();
      transformStamped_source.transform.translation.z = loop->key2->node->estimate().translation().z();
      transformStamped_source.transform.rotation.x    = quat_source.x();
      transformStamped_source.transform.rotation.y    = quat_source.y();
      transformStamped_source.transform.rotation.z    = quat_source.z();
      transformStamped_source.transform.rotation.w    = quat_source.w();
      transformStamped_source.header.stamp = ros::Time::now();

      debug_tf2_tf_broadcaster.sendTransform(transformStamped_target);
      debug_tf2_tf_broadcaster.sendTransform(transformStamped_source);

      // Target PointCloud
      sensor_msgs::PointCloud2Ptr cloud_msg_target(new sensor_msgs::PointCloud2());
      pcl::toROSMsg(*loop->key1->cloud, *cloud_msg_target);
      cloud_msg_target->header.frame_id = "debug_loop_closure_target_pose";
      cloud_msg_target->header.stamp = ros::Time::now();
      debug_loop_closer_target_pub.publish(cloud_msg_target);
      // Source PointCloud
      sensor_msgs::PointCloud2Ptr cloud_msg_source(new sensor_msgs::PointCloud2());
      pcl::toROSMsg(*loop->key2->cloud, *cloud_msg_source);
      cloud_msg_source->header.frame_id = "debug_loop_closure_source_pose";
      cloud_msg_source->header.stamp = ros::Time::now();
      debug_loop_closer_source_pub.publish(cloud_msg_source);
    }

    /**
     * @brief create visualization marker
     * @param stamp
     * @return
     */
    visualization_msgs::MarkerArray create_marker_array(const ros::Time &stamp) const
    {
      visualization_msgs::MarkerArray markers;
      markers.markers.resize(4);

      // node markers
      visualization_msgs::Marker &traj_marker = markers.markers[0];
      traj_marker.header.frame_id = "map";
      traj_marker.header.stamp = stamp;
      traj_marker.ns = "nodes";
      traj_marker.id = 0;
      traj_marker.type = visualization_msgs::Marker::SPHERE_LIST;

      traj_marker.pose.orientation.w = 1.0;
      traj_marker.scale.x = traj_marker.scale.y = traj_marker.scale.z = 0.25;

      // As I don't use imu data, I don't need to visualize it.
      // visualization_msgs::Marker &imu_marker = markers.markers[1];
      // imu_marker.header = traj_marker.header;
      // imu_marker.ns = "imu";
      // imu_marker.id = 1;
      // imu_marker.type = visualization_msgs::Marker::SPHERE_LIST;

      // imu_marker.pose.orientation.w = 1.0;
      // imu_marker.scale.x = imu_marker.scale.y = imu_marker.scale.z = 0.75;

      traj_marker.points.resize(keyframes.size());
      traj_marker.colors.resize(keyframes.size());
      for (int i = 0; i < keyframes.size(); i++)
      {
        Eigen::Vector3d pos = keyframes[i]->node->estimate().translation();
        traj_marker.points[i].x = pos.x();
        traj_marker.points[i].y = pos.y();
        traj_marker.points[i].z = pos.z();

        double p = static_cast<double>(i) / keyframes.size();
        traj_marker.colors[i].r = 1.0 - p;
        traj_marker.colors[i].g = p;
        traj_marker.colors[i].b = 0.0;
        traj_marker.colors[i].a = 1.0;

        if (keyframes[i]->acceleration)
        {
          Eigen::Vector3d pos = keyframes[i]->node->estimate().translation();
          geometry_msgs::Point point;
          point.x = pos.x();
          point.y = pos.y();
          point.z = pos.z();

          std_msgs::ColorRGBA color;
          color.r = 0.0;
          color.g = 0.0;
          color.b = 1.0;
          color.a = 0.1;

          // imu_marker.points.push_back(point);
          // imu_marker.colors.push_back(color);
        }
      }

      // edge markers
      visualization_msgs::Marker &edge_marker = markers.markers[2];
      edge_marker.header.frame_id = "map";
      edge_marker.header.stamp = stamp;
      edge_marker.ns = "edges";
      edge_marker.id = 2;
      edge_marker.type = visualization_msgs::Marker::LINE_LIST;

      edge_marker.pose.orientation.w = 1.0;
      edge_marker.scale.x = 0.05;

      edge_marker.points.resize(graph_slam->graph->edges().size() * 2);
      edge_marker.colors.resize(graph_slam->graph->edges().size() * 2);

      auto edge_itr = graph_slam->graph->edges().begin();
      for (int i = 0; edge_itr != graph_slam->graph->edges().end(); edge_itr++, i++)
      {
        g2o::HyperGraph::Edge *edge = *edge_itr;
        g2o::EdgeSE3 *edge_se3 = dynamic_cast<g2o::EdgeSE3 *>(edge);
        if (edge_se3)
        {
          g2o::VertexSE3 *v1 = dynamic_cast<g2o::VertexSE3 *>(edge_se3->vertices()[0]);
          g2o::VertexSE3 *v2 = dynamic_cast<g2o::VertexSE3 *>(edge_se3->vertices()[1]);
          Eigen::Vector3d pt1 = v1->estimate().translation();
          Eigen::Vector3d pt2 = v2->estimate().translation();

          edge_marker.points[i * 2].x = pt1.x();
          edge_marker.points[i * 2].y = pt1.y();
          edge_marker.points[i * 2].z = pt1.z();
          edge_marker.points[i * 2 + 1].x = pt2.x();
          edge_marker.points[i * 2 + 1].y = pt2.y();
          edge_marker.points[i * 2 + 1].z = pt2.z();

          double p1 = static_cast<double>(v1->id()) / graph_slam->graph->vertices().size();
          double p2 = static_cast<double>(v2->id()) / graph_slam->graph->vertices().size();
          edge_marker.colors[i * 2].r = 1.0 - p1;
          edge_marker.colors[i * 2].g = p1;
          edge_marker.colors[i * 2].a = 1.0;
          edge_marker.colors[i * 2 + 1].r = 1.0 - p2;
          edge_marker.colors[i * 2 + 1].g = p2;
          edge_marker.colors[i * 2 + 1].a = 1.0;

          if (std::abs(v1->id() - v2->id()) > 2)
          {
            edge_marker.points[i * 2].z += 0.5;
            edge_marker.points[i * 2 + 1].z += 0.5;
          }

          continue;
        }

        g2o::EdgeSE3Plane *edge_plane = dynamic_cast<g2o::EdgeSE3Plane *>(edge);
        if (edge_plane)
        {
          g2o::VertexSE3 *v1 = dynamic_cast<g2o::VertexSE3 *>(edge_plane->vertices()[0]);
          Eigen::Vector3d pt1 = v1->estimate().translation();
          Eigen::Vector3d pt2(pt1.x(), pt1.y(), 0.0);

          edge_marker.points[i * 2].x = pt1.x();
          edge_marker.points[i * 2].y = pt1.y();
          edge_marker.points[i * 2].z = pt1.z();
          edge_marker.points[i * 2 + 1].x = pt2.x();
          edge_marker.points[i * 2 + 1].y = pt2.y();
          edge_marker.points[i * 2 + 1].z = pt2.z();

          edge_marker.colors[i * 2].b = 1.0;
          edge_marker.colors[i * 2].a = 1.0;
          edge_marker.colors[i * 2 + 1].b = 1.0;
          edge_marker.colors[i * 2 + 1].a = 1.0;

          continue;
        }

        g2o::EdgeSE3PriorXY *edge_priori_xy = dynamic_cast<g2o::EdgeSE3PriorXY *>(edge);
        if (edge_priori_xy)
        {
          g2o::VertexSE3 *v1 = dynamic_cast<g2o::VertexSE3 *>(edge_priori_xy->vertices()[0]);
          Eigen::Vector3d pt1 = v1->estimate().translation();
          Eigen::Vector3d pt2 = Eigen::Vector3d::Zero();
          pt2.head<2>() = edge_priori_xy->measurement();

          edge_marker.points[i * 2].x = pt1.x();
          edge_marker.points[i * 2].y = pt1.y();
          edge_marker.points[i * 2].z = pt1.z() + 0.5;
          edge_marker.points[i * 2 + 1].x = pt2.x();
          edge_marker.points[i * 2 + 1].y = pt2.y();
          edge_marker.points[i * 2 + 1].z = pt2.z() + 0.5;

          edge_marker.colors[i * 2].r = 1.0;
          edge_marker.colors[i * 2].a = 1.0;
          edge_marker.colors[i * 2 + 1].r = 1.0;
          edge_marker.colors[i * 2 + 1].a = 1.0;

          continue;
        }

        g2o::EdgeSE3PriorXYZ *edge_priori_xyz = dynamic_cast<g2o::EdgeSE3PriorXYZ *>(edge);
        if (edge_priori_xyz)
        {
          g2o::VertexSE3 *v1 = dynamic_cast<g2o::VertexSE3 *>(edge_priori_xyz->vertices()[0]);
          Eigen::Vector3d pt1 = v1->estimate().translation();
          Eigen::Vector3d pt2 = edge_priori_xyz->measurement();

          edge_marker.points[i * 2].x = pt1.x();
          edge_marker.points[i * 2].y = pt1.y();
          edge_marker.points[i * 2].z = pt1.z() + 0.5;
          edge_marker.points[i * 2 + 1].x = pt2.x();
          edge_marker.points[i * 2 + 1].y = pt2.y();
          edge_marker.points[i * 2 + 1].z = pt2.z();

          edge_marker.colors[i * 2].r = 1.0;
          edge_marker.colors[i * 2].a = 1.0;
          edge_marker.colors[i * 2 + 1].r = 1.0;
          edge_marker.colors[i * 2 + 1].a = 1.0;

          continue;
        }
      }

      // sphere
      visualization_msgs::Marker &sphere_marker = markers.markers[3];
      sphere_marker.header.frame_id = "map";
      sphere_marker.header.stamp = stamp;
      sphere_marker.ns = "loop_close_radius";
      sphere_marker.id = 3;
      sphere_marker.type = visualization_msgs::Marker::SPHERE;

      if (!keyframes.empty())
      {
        Eigen::Vector3d pos = keyframes.back()->node->estimate().translation();
        sphere_marker.pose.position.x = pos.x();
        sphere_marker.pose.position.y = pos.y();
        sphere_marker.pose.position.z = pos.z();
      }
      sphere_marker.pose.orientation.w = 1.0;
      sphere_marker.scale.x = sphere_marker.scale.y = sphere_marker.scale.z = loop_detector->get_distance_thresh() * 2.0;

      sphere_marker.color.r = 1.0;
      sphere_marker.color.a = 0.3;

      return markers;
    }

    /**
     * @brief load all data from a directory
     * @param req
     * @param res
     * @return
     */
    bool load_service(hdl_graph_slam::LoadGraphRequest &req, hdl_graph_slam::LoadGraphResponse &res)
    {
      std::lock_guard<std::mutex> lock(main_thread_mutex);

      std::string directory = req.path;

      std::cout << "loading data from:" << directory << std::endl;

      // Load graph.
      graph_slam->load(directory + "/graph.g2o");

      // Iterate over the items in this directory and count how many sub directories there are.
      // This will give an upper limit on how many keyframe indexes we can expect to find.
      boost::filesystem::directory_iterator begin(directory), end;
      int max_directory_count = std::count_if(begin, end,
                                              [](const boost::filesystem::directory_entry &d)
                                              {
                                                return boost::filesystem::is_directory(d.path()); // only return true if a direcotry
                                              });

      // Load keyframes by looping through key frame indexes that we expect to see.
      for (int i = 0; i < max_directory_count; i++)
      {
        std::stringstream sst;
        sst << boost::format("%s/%06d") % directory % i;
        std::string key_frame_directory = sst.str();

        // If key_frame_directory doesnt exist, then we have run out so lets stop looking.
        if (!boost::filesystem::is_directory(key_frame_directory))
        {
          break;
        }

        KeyFrame::Ptr keyframe(new KeyFrame(key_frame_directory, graph_slam->graph.get()));
        keyframes.push_back(keyframe);
      }
      std::cout << "loaded " << keyframes.size() << " keyframes" << std::endl;

      // Load special nodes.
      std::ifstream ifs(directory + "/special_nodes.csv");
      if (!ifs)
      {
        return false;
      }
      while (!ifs.eof())
      {
        std::string token;
        ifs >> token;
        if (token == "anchor_node")
        {
          int id = 0;
          ifs >> id;
          anchor_node = static_cast<g2o::VertexSE3 *>(graph_slam->graph->vertex(id));
        }
        else if (token == "anchor_edge")
        {
          int id = 0;
          ifs >> id;
          // We have no way of directly pulling the edge based on the edge ID that we have just read in.
          // Fortunatly anchor edges are always attached to the anchor node so we can iterate over
          // the edges that listed against the node untill we find the one that matches our ID.
          if (anchor_node)
          {
            auto edges = anchor_node->edges();

            for (auto e : edges)
            {
              int edgeID = e->id();
              if (edgeID == id)
              {
                anchor_edge = static_cast<g2o::EdgeSE3 *>(e);

                break;
              }
            }
          }
        }
        else if (token == "floor_node")
        {
          int id = 0;
          ifs >> id;
          floor_plane_node = static_cast<g2o::VertexPlane *>(graph_slam->graph->vertex(id));
        }
      }

      // check if we have any non null special nodes, if all are null then lets not bother.
      if (anchor_node->id() || anchor_edge->id() || floor_plane_node->id())
      {
        std::cout << "loaded special nodes - ";

        // check exists before printing information about each special node
        if (anchor_node->id())
        {
          std::cout << " anchor_node: " << anchor_node->id();
        }
        if (anchor_edge->id())
        {
          std::cout << " anchor_edge: " << anchor_edge->id();
        }
        if (floor_plane_node->id())
        {
          std::cout << " floor_node: " << floor_plane_node->id();
        }

        // finish with a new line
        std::cout << std::endl;
      }

      // Update our keyframe snapshot so we can publish a map update, trigger update with graph_updated = true.
      std::vector<KeyFrameSnapshot::Ptr> snapshot(keyframes.size());

      std::transform(keyframes.begin(), keyframes.end(), snapshot.begin(), [=](const KeyFrame::Ptr &k)
                     { return std::make_shared<KeyFrameSnapshot>(k); });

      keyframes_snapshot_mutex.lock();
      keyframes_snapshot.swap(snapshot);
      keyframes_snapshot_mutex.unlock();
      graph_updated = true;

      res.success = true;

      std::cout << "snapshot updated" << std::endl
                << "loading successful" << std::endl;

      return true;
    }

    /**
     * @brief dump all data to the current directory
     * @param req
     * @param res
     * @return
     */
    bool dump_service(hdl_graph_slam::DumpGraphRequest &req, hdl_graph_slam::DumpGraphResponse &res)
    {
      std::lock_guard<std::mutex> lock(main_thread_mutex);

      std::string directory = req.destination;

      if (directory.empty())
      {
        std::array<char, 64> buffer;
        buffer.fill(0);
        time_t rawtime;
        time(&rawtime);
        const auto timeinfo = localtime(&rawtime);
        strftime(buffer.data(), sizeof(buffer), "%d-%m-%Y %H:%M:%S", timeinfo);
      }

      if (!boost::filesystem::is_directory(directory))
      {
        boost::filesystem::create_directory(directory);
      }

      std::cout << "dumping data to:" << directory << std::endl;
      // save graph
      graph_slam->save(directory + "/graph.g2o");

      // save keyframes
      for (int i = 0; i < keyframes.size(); i++)
      {
        std::stringstream sst;
        sst << boost::format("%s/%06d") % directory % i;

        keyframes[i]->save(sst.str());
      }

      if (zero_utm)
      {
        std::ofstream zero_utm_ofs(directory + "/zero_utm");
        zero_utm_ofs << boost::format("%.6f %.6f %.6f") % zero_utm->x() % zero_utm->y() % zero_utm->z() << std::endl;
      }

      std::ofstream ofs(directory + "/special_nodes.csv");
      ofs << "anchor_node " << (anchor_node == nullptr ? -1 : anchor_node->id()) << std::endl;
      ofs << "anchor_edge " << (anchor_edge == nullptr ? -1 : anchor_edge->id()) << std::endl;
      ofs << "floor_node " << (floor_plane_node == nullptr ? -1 : floor_plane_node->id()) << std::endl;

      res.success = true;
      return true;
    }

    /**
     * @brief save map data as pcd
     * @param req
     * @param res
     * @return
     */
    bool save_map_service(hdl_graph_slam::SaveMapRequest &req, hdl_graph_slam::SaveMapResponse &res)
    {
      std::vector<KeyFrameSnapshot::Ptr> snapshot;

      keyframes_snapshot_mutex.lock();
      snapshot = keyframes_snapshot;
      keyframes_snapshot_mutex.unlock();

      auto cloud = map_cloud_generator->generate(snapshot, req.resolution);
      if (!cloud)
      {
        res.success = false;
        return true;
      }

      if (zero_utm && req.utm)
      {
        for (auto &pt : cloud->points)
        {
          pt.getVector3fMap() += (*zero_utm).cast<float>();
        }
      }

      cloud->header.frame_id = map_frame_id;
      cloud->header.stamp = snapshot.back()->cloud->header.stamp;

      if (zero_utm)
      {
        std::ofstream ofs(req.destination + ".utm");
        ofs << boost::format("%.6f %.6f %.6f") % zero_utm->x() % zero_utm->y() % zero_utm->z() << std::endl;
      }

      int ret = pcl::io::savePCDFileBinary(req.destination, *cloud);
      res.success = ret == 0;

      return true;
    }

  private:
    // Debug Variables
    ros::Publisher debug_loop_closer_target_pub;
    ros::Publisher debug_loop_closer_source_pub;
    std::mutex debug_loop_closure_keyframe_mutex; // 솔직히 이걸 뮤택스로 막아줘야하는지 잘 모르겠지만 일딴 써
    ros::Publisher debug_loop_closure_target_pose_pub;
    ros::Publisher debug_loop_closure_source_pose_pub;
    tf2_ros::TransformBroadcaster debug_tf2_tf_broadcaster;

    



    // ROS
    ros::NodeHandle nh;
    ros::NodeHandle mt_nh;
    ros::NodeHandle private_nh;
    ros::WallTimer optimization_timer;
    ros::WallTimer map_publish_timer;

    std::unique_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> odom_sub;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub;
    std::unique_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync;

    ros::Subscriber gps_sub;
    ros::Subscriber nmea_sub;
    ros::Subscriber navsat_sub;

    ros::Subscriber imu_sub;
    ros::Subscriber floor_sub;

    ros::Publisher markers_pub;

    std::string published_odom_topic;
    std::string map_frame_id;
    std::string odom_frame_id;

    std::mutex trans_odom2map_mutex;
    Eigen::Matrix4f trans_odom2map;
    ros::Publisher odom2map_pub;

    std::string points_topic;
    ros::Publisher read_until_pub;
    ros::Publisher map_points_pub;

    tf::TransformListener tf_listener;

    ros::ServiceServer load_service_server;
    ros::ServiceServer dump_service_server;
    ros::ServiceServer save_map_service_server;

    // keyframe queue
    std::string base_frame_id;
    std::mutex keyframe_queue_mutex;
    std::deque<KeyFrame::Ptr> keyframe_queue;

    // gps queue
    double gps_time_offset;
    double gps_edge_stddev_xy;
    double gps_edge_stddev_z;
    boost::optional<Eigen::Vector3d> zero_utm;
    std::mutex gps_queue_mutex;
    std::deque<geographic_msgs::GeoPointStampedConstPtr> gps_queue;

    // imu queue
    double imu_time_offset;
    bool enable_imu_orientation;
    double imu_orientation_edge_stddev;
    bool enable_imu_acceleration;
    double imu_acceleration_edge_stddev;
    std::mutex imu_queue_mutex;
    std::deque<sensor_msgs::ImuConstPtr> imu_queue;

    // floor_coeffs queue
    double floor_edge_stddev;
    std::mutex floor_coeffs_queue_mutex;
    std::deque<hdl_graph_slam::FloorCoeffsConstPtr> floor_coeffs_queue;

    // for map cloud generation
    std::atomic_bool graph_updated;
    double map_cloud_resolution;
    std::mutex keyframes_snapshot_mutex;
    std::vector<KeyFrameSnapshot::Ptr> keyframes_snapshot;
    std::unique_ptr<MapCloudGenerator> map_cloud_generator;

    // graph slam
    // all the below members must be accessed after locking main_thread_mutex
    std::mutex main_thread_mutex;

    int max_keyframes_per_update;
    std::deque<KeyFrame::Ptr> new_keyframes;

    g2o::VertexSE3 *anchor_node;
    g2o::EdgeSE3 *anchor_edge;
    g2o::VertexPlane *floor_plane_node;
    std::vector<KeyFrame::Ptr> keyframes;
    std::unordered_map<ros::Time, KeyFrame::Ptr, RosTimeHash> keyframe_hash;

    std::unique_ptr<GraphSLAM> graph_slam;
    std::unique_ptr<LoopDetector> loop_detector;
    std::unique_ptr<KeyframeUpdater> keyframe_updater;
    std::unique_ptr<NmeaSentenceParser> nmea_parser;

    std::unique_ptr<InformationMatrixCalculator> inf_calclator;
  };

} // namespace hdl_graph_slam

PLUGINLIB_EXPORT_CLASS(hdl_graph_slam::HdlGraphSlamNodelet, nodelet::Nodelet)
