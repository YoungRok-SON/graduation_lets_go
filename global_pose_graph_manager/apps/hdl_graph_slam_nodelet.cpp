// SPDX-License-Identifier: BSD-2-Clause

#include <ctime>
#include <mutex>
#include <atomic>
#include <memory>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>


#include <ros/ros.h>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <pcl_ros/point_cloud.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <std_msgs/Time.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include "keyframe_msgs/keyframe.h"
#include "keyframe_msgs/updatedKeyFrame.h"

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

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <g2o/edge_se3_priorvec.hpp>
#include <g2o/edge_se3_priorquat.hpp>

// For New Features
#include <pclomp/ndt_omp.h>
#include <pclomp/voxel_grid_covariance_omp.h>

namespace hdl_graph_slam
{

  class HdlGraphSlamNodelet : public nodelet::Nodelet
  {
  public:
    typedef pcl::PointXYZI PointT;
    typedef pcl::PointXYZRGB PointC;
    typedef pclomp::VoxelGridCovariance<PointT>::Leaf Leaf;
    typedef std::map<size_t, pclomp::VoxelGridCovariance<PointT>::Leaf> LeafMap;

    HdlGraphSlamNodelet() {}
    virtual ~HdlGraphSlamNodelet() {}

    virtual void onInit()
    {
      nh = getNodeHandle();
      mt_nh = getMTNodeHandle();
      private_nh = getPrivateNodeHandle();

      // init parameters
      // published_odom_topic = private_nh.param<std::string>("published_odom_topic", "/odom");
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
      graph_slam.reset(new GraphSLAM(private_nh.param<std::string>("g2o_solver_type", "lm_var")));

      // Graph SLAM Functionalities
      keyframe_updater.reset(new KeyframeUpdater(private_nh));
      loop_detector.reset(new LoopDetector(private_nh)); // Loop Closure 클래스 객체 생성
      map_cloud_generator.reset(new MapCloudGenerator());
      inf_calclator.reset(new InformationMatrixCalculator(private_nh));

      points_topic = private_nh.param<std::string>("points_topic", "/velodyne_points");

      /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      /* Debugging Publisher for Loop Closure */
      debug_loop_closer_aligned_pub      = nh.advertise<sensor_msgs::PointCloud2>("/hdl_graph_slam/debug/loop_closer_aligned", 1, true);
      debug_loop_closer_target_pub       = nh.advertise<sensor_msgs::PointCloud2>("/hdl_graph_slam/debug/loop_closer_target", 1, true);
      debug_loop_closer_source_pub       = nh.advertise<sensor_msgs::PointCloud2>("/hdl_graph_slam/debug/loop_closer_source", 1, true);
      debug_loop_closure_target_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/hdl_graph_slam/debug/loop_closure_target_pose", 1, true);
      debug_loop_closure_source_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/hdl_graph_slam/debug/loop_closure_source_pose", 1, true);

      /* Publisher for Debugging  NDT Things */
      // Maybe these topics can be changed to mt_nh
      debug_ndt_scan_arrow_marker_pub    = nh.advertise<visualization_msgs::MarkerArray>("/hdl_graph_slam/debug/ndt_scan_arrow", 1, true);
      debug_ndt_scan_marker_pub          = nh.advertise<visualization_msgs::MarkerArray>("/hdl_graph_slam/debug/ndt_scan", 1, true);
      debug_ndt_map_marker_pub           = nh.advertise<visualization_msgs::MarkerArray>("/hdl_graph_slam/debug/ndt_map", 1, true);
      debug_loop_closure_sub_map_pub     = nh.advertise<visualization_msgs::MarkerArray>("/hdl_graph_slam/debug/loop_closure_sub_map", 1, true);

      /* Variables for NDT things */
      leaf_voxel_size_    = private_nh.param<double>("leaf_voxel_size", 0.5);
      create_scan_ndt_    = private_nh.param<bool>("create_scan_ndt", false);
      min_nr_             = private_nh.param<int>("min_nr", 10);
      ndt_leaf_min_scale_ = private_nh.param<double>("ndt_leaf_min_scale", 0.01);
      use_submap_loop_    = private_nh.param<bool>("use_submap_loop", false);

      // Keyframe Subscriber
      keyframe_sub_ = nh.subscribe("/orb_slam2_rgbd/keyframe", 16, &HdlGraphSlamNodelet::keyframe_callback, this);
      // Service server for updatedKeyframe resquest -> callback function needs to be lock when keyframe's pose is updated.
      updated_keyframe_client_ = mt_nh.advertiseService("orb_slam2_ros/updatedKeyframe", &HdlGraphSlamNodelet::updated_keyframe_callback, this);
      num_vehicle_  = private_nh.param<int>("num_vehicle", 2);
      /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


      // Publishers
      markers_pub    = mt_nh.advertise<visualization_msgs::MarkerArray>("/hdl_graph_slam/markers", 16);
      odom2map_pub   = mt_nh.advertise<geometry_msgs::TransformStamped>("/hdl_graph_slam/odom2pub", 16);
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
      keyframe_queues.resize(num_vehicle_); // 기체 수만큼 큐를 벡터에 생성
    }

  private:

    void keyframe_callback(const keyframe_msgs::keyframe &keyframe_msg)
    {
      // Get odom pose
      const ros::Time &stamp       = keyframe_msg.header.stamp;
      Eigen::Isometry3d odom = pose2isometry(keyframe_msg.Pose);
      
      // Get Point Cloud Data
      pcl::PointCloud<PointC>::Ptr cloud(new pcl::PointCloud<PointC>());
      pcl::fromROSMsg(keyframe_msg.PointCloud, *cloud);
      
      // Get Keyframe id and vehicle id
      const int &keyframe_id       = keyframe_msg.id;
      const int &vehicle_id        = keyframe_msg.vehicle_id;

      // 이전 keyframe과의 거리 계산을 통해 짧으면 false, 길면 true
      if (!keyframe_updater->update(odom)) 
        return;

      // Create Keyframe object
      double accum_d = keyframe_updater->get_accum_distance();
      KeyFrame::Ptr keyframe = KeyFrame::Ptr(new KeyFrame(stamp, odom, accum_d, cloud, keyframe_id, vehicle_id));

      std::lock_guard<std::mutex> lock(keyframe_queue_mutex); // 뮤텍스 걸고
      keyframe_queues[keyframe->vehicle_id].push_back(keyframe); // 생성한 키프레임 객체를 id에 맞는 큐에 넣기
    }

    bool updated_keyframe_callback(keyframe_msgs::updatedKeyFrame::Request &req, keyframe_msgs::updatedKeyFrame::Response &res)
    {
      std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
      // Below code is needed to change to change keyframe pose.
      // It is send to server as ORB_SLAM2 Bundle Adjustment has updated graph.
      if (keyframe_queue.empty())
      {
        // res.updated_keyframe_id = -1;
        return true;
      }
      // res.updated_keyframe_id = keyframe_queue.back()->id;
      return false;
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
      // trans_odom2map은 어디서 가져오는거? -> 제일 마지막으로 입력된 키프레임의 최적화된 포즈와 마지막으로 들어온 오도메트리 포즈 사이의 변환값 
      // 이 값을 이용해서 오돔 좌표를 그래프 좌표로 보정해줌(drift 보정이라고 생각하면 편할듯?)
      trans_odom2map_mutex.unlock();

      std::cout << "flush_keyframe_queue - keyframes len:" << keyframes.size() << std::endl; 
      //Keyframe queue도 있는데 이건 왜 따로 벡터로 지정해놨지? -> keyframe_queue는 callback에서 계속 쌓이고 이 함수 마지막에 그래프에 들어간 keyframe만큼 지워짐. 그리고 keyframes 벡터에 넣어줌.
      int num_processed = 0;
      for (int i = 0; i < std::min<int>(keyframe_queue.size(), max_keyframes_per_update); i++) // Keyframe queue에 있는 개수가 min보다 작으면 개수만큼만 진행
      {
        num_processed = i;

        const auto &keyframe = keyframe_queue[i];
        // new_keyframes will be tested later for loop closure
        new_keyframes.push_back(keyframe);

        // add pose node
        Eigen::Isometry3d odom = odom2map * keyframe->odom;
        // Keyframe 객체를 생성할 때 노드에 대한 값이 비어 있음. 그걸 이때 채워넣음. 그리고 그래프 안에도 넣어줌
        keyframe->node = graph_slam->add_se3_node(odom); 
        
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
            // 처음 노드
            anchor_node = graph_slam->add_se3_node(Eigen::Isometry3d::Identity()); 
            anchor_node->setFixed(true);  // 고정
            anchor_edge = graph_slam->add_se3_edge(anchor_node, keyframe->node, Eigen::Isometry3d::Identity(), inf); // 앵커노드와 첫번째 키프레임 사이 에지를 지정
          }
        }

        if (i == 0 && keyframes.empty()) // 만약 Keyframe들을 모아놓는 벡터에 아무것도 없으면 그냥 넘어감
        {
          continue; // 이 std::vector<KeyFrame::Ptr> 형태의 변수 용도가 뭔지? -> 키프레임들을 모아놓는 벡터
        }

        /* --------------------------------------------------------------------------- */
        /* 이 부분에 vehicle number와 관련해서 마지막 부분을 어떻게 이어줄 지 로직을 짜야함 */
        // add edge between consecutive keyframes
        const auto &prev_keyframe = i == 0 ? keyframes.back() : keyframe_queue[i - 1]; // 일단 i==0 일때 현재 키프레임과 이전 키프레임을 엮어줘야하니 prev_keyframe을 지정해주는 것으로 보임
        /* --------------------------------------------------------------------------- */

        // 연결된 두 포즈 사이의 상대 포즈를 계산
        Eigen::Isometry3d relative_pose = keyframe->odom.inverse() * prev_keyframe->odom; // 이전 키프레임 -> 현재 키프레임으로 가는 상대 포즈
        // 연결된 두 포주 사이의 정보 행렬 계산 -> 고정된 값으로 진행(ORB), 조금 더 큰 값(NDT), 아루코마커 위치 정보에 대한 정보 행렬도 지정해서 그냥 사용
        Eigen::MatrixXd   information   = inf_calclator->calc_information_matrix( keyframe->cloud_t, prev_keyframe->cloud_t, relative_pose);
        // 연결된 두 포즈 사이의 에지를 추가
        auto edge = graph_slam->add_se3_edge(keyframe->node, prev_keyframe->node, relative_pose, information);
        // Robust Kernel 추가
        graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("odometry_edge_robust_kernel", "NONE"), private_nh.param<double>("odometry_edge_robust_kernel_size", 1.0));
      }

      // 3초에 한번씩 도니까 한번 싹 밀어 넣은 다음에 처리한건 다 지움
      keyframe_queue.erase(keyframe_queue.begin(), keyframe_queue.begin() + num_processed + 1);
      return true;
    }

    /**
     * @brief generate map point cloud and publish it
     * @param event
     */
    void map_points_publish_timer_callback(const ros::WallTimerEvent &event)
    {
      if ( !(map_points_pub.getNumSubscribers() || debug_ndt_map_marker_pub.getNumSubscribers())  || !graph_updated )
      {
        return;
      }
      ROS_INFO("Generate Map Cloud");
      std::vector<KeyFrameSnapshot::Ptr> snapshot;

      keyframes_snapshot_mutex.lock();
      snapshot = keyframes_snapshot; // 처음 키프레임 ~ 현제까지 flush된 키프레임 집합
      keyframes_snapshot_mutex.unlock();

      auto cloud = map_cloud_generator->generate(snapshot, map_cloud_resolution);
      if (!cloud)
      {
        return;
      }

      cloud->header.frame_id = map_frame_id;
      cloud->header.stamp = snapshot.back()->cloud->header.stamp;

      ROS_INFO("Map Size: %ld", cloud->size());
      
      // To Do : 전체 맵을 다 볼 수 있는 기능 넣기
      // To Do : subscribe하는 경우에만 만들기
      // To Do : Topic명 스캔, 서브맵, 전체맵에 대해 변수명 잘 구별해주기

      // NDT Leaf Visualization
      // if(debug_ndt_map_marker_pub)
      // {
      //   ROS_INFO("Generate NDT Leaves");
      //   // Voxel Grid Covariance
      //   // 요놈이 문제 -> 이 필터에 들어가기 전까지는 cloud도 정상적인데 
      //   pclomp::VoxelGridCovariance<PointT> generate_ndt_scan;
      //   generate_ndt_scan.setInputCloud(cloud);
      //   generate_ndt_scan.setLeafSize(leaf_voxel_size_, leaf_voxel_size_, leaf_voxel_size_);
      //   generate_ndt_scan.setMinPointPerVoxel(10);
      //   generate_ndt_scan.filter();
      //   LeafMap leaves = generate_ndt_scan.getLeaves();
      //   ROS_INFO("NDT Map Size: %ld", leaves.size());

      //   // create_maker_array_ndt_whole_map() 넣기
      //   create_marker_array_ndt(cloud, leaves);
        
      //   ROS_INFO("Marker Publish Done");
      // }
      sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2()); 
      pcl::toROSMsg(*cloud, *cloud_msg);  // 실행하면 cloud가 비워짐.
      map_points_pub.publish(cloud_msg);
    }

      
    /**
     * @brief this methods adds all the data in the queues to the pose graph, and then optimizes the pose graph
     * @param event
     */
    void optimization_timer_callback(const ros::WallTimerEvent &event)
    {
      std::lock_guard<std::mutex> lock(main_thread_mutex);

      // add keyframes in the queues to the pose graph
      bool keyframe_updated = flush_keyframe_queue(); // for local optimization

      if (!keyframe_updated)
        return;

      // loop detection ->
      std::vector<Loop::Ptr> loops = loop_detector->detect(keyframes, new_keyframes, *graph_slam);
      // 새로운 키프래임과 이전 키프레임 사이의 매칭 진행? -> N(기존 키프레임들) : M(새로운 키프레임들) 사이의 매칭을 진행해서 여러개가 나올 수 있음.
      for (const auto &loop : loops)
      {
        Eigen::Isometry3d relpose(loop->relative_pose.cast<double>());
        Eigen::MatrixXd information_matrix = inf_calclator->calc_information_matrix(loop->key1->cloud_t, loop->key2->cloud_t, relpose);
        auto edge = graph_slam->add_se3_edge(loop->key1->node, loop->key2->node, relpose, information_matrix); // 여기서 에러 나는 거 같은디
        
        graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("loop_closure_edge_robust_kernel", "NONE"), private_nh.param<double>("loop_closure_edge_robust_kernel_size", 1.0));
        
        // Debug Publisher
        
        if(debug_loop_closer_target_pub.getNumSubscribers() && debug_loop_closer_source_pub.getNumSubscribers())
        {
          debug_loop_closure_points_pose(loop);
        }
        // if(debug_loop_closure_sub_map_pub.getNumSubscribers())
        {
          // NDT Sub Map Visulization
        }
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
      Eigen::Isometry3d trans = keyframe->node->estimate() * keyframe->odom.inverse(); // 이건 어떤 값? -> 오도메트리와 계산된 그래프를 기반으로 최적화된 위치 결과 사이의 변환, 결국 누적 오차 없에는 역할
      trans_odom2map_mutex.lock();
      trans_odom2map = trans.matrix().cast<float>(); // 이 값은 왜 이렇게 해놓지?-> 다음  keyframe이 들어올 때 이전 keyframe과의 차이를 계산하기 위해서
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
      pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_cloud;
      Eigen::Matrix4f final_transformation;
      loop_detector->getAlignedCloudWithFianlTF(aligned_cloud, final_transformation);
      std::cout <<"aligned PCD Size: " << aligned_cloud->size() << std::endl;
      std::cout <<"Relentive Pose: " << final_transformation << std::endl;
      // Source PointCloud
      sensor_msgs::PointCloud2Ptr aligned_cloud_msg_source(new sensor_msgs::PointCloud2());
      pcl::toROSMsg(*aligned_cloud, *aligned_cloud_msg_source);
      aligned_cloud_msg_source->header.frame_id = map_frame_id;
      aligned_cloud_msg_source->header.stamp = ros::Time::now();
      debug_loop_closer_aligned_pub.publish(aligned_cloud_msg_source);

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

      // It need to be map center... when use a submap
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
      pcl::toROSMsg(*loop->key1->cloud_c, *cloud_msg_target);
      if(use_submap_loop_)
        cloud_msg_target->header.frame_id = "map";
      else
        cloud_msg_target->header.frame_id = "debug_loop_closure_target_pose";
      cloud_msg_target->header.stamp = ros::Time::now();
      debug_loop_closer_target_pub.publish(cloud_msg_target);
      // Source PointCloud
      sensor_msgs::PointCloud2Ptr cloud_msg_source(new sensor_msgs::PointCloud2());
      pcl::toROSMsg(*loop->key2->cloud_c, *cloud_msg_source);
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
     * @brief create visualization marker of SINGLE keyframe for NDT Leafs
     * @param keyframes - keyframes to visualize as markers
     * @return visualization_msgs::MarkerArray
     */
    void create_marker_array_ndt(KeyFrame::Ptr keyframe)
    {
        // Generate 
        visualization_msgs::MarkerArrayPtr marker_array(new visualization_msgs::MarkerArray());
        visualization_msgs::MarkerArrayPtr marker_array_arrow(new visualization_msgs::MarkerArray());
        visualization_msgs::Marker marker;
        visualization_msgs::Marker marker_arrow;
        marker.header.frame_id = "base_link";
        marker_arrow.header.frame_id = "base_link";
        marker.header.stamp = ros::Time(keyframe->cloud_t->header.stamp/1000000, (keyframe->cloud_t->header.stamp%1000000) * 1000); // pcl timestamp to ros timestamp
        marker_arrow.header.stamp = ros::Time(keyframe->cloud_t->header.stamp/1000000, (keyframe->cloud_t->header.stamp%1000000) * 1000); // pcl timestamp to ros timestamp
        marker.ns = "ndt_map_scan";
        marker_arrow.ns = "ndt_map_scan";
        
        marker.type = visualization_msgs::Marker::SPHERE;
        marker_arrow.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker_arrow.action = visualization_msgs::Marker::ADD;
        int marker_id = 0;
        

        // Set markers for each leaf
        for (const auto &leaf : keyframe->leaves)
        {
          // Get a properties of Leaf
          auto mean = leaf.second.getMean();
          auto cov = leaf.second.getCov();
          auto eigen_vectors = leaf.second.getEvecs();
          auto eigen_values = leaf.second.getEvals();

          // Set properties
          marker.id = marker_id++; 
          marker.lifetime = ros::Duration(0.0);
          marker.frame_locked = true;
          marker.color.r = 0.5;
          marker.color.g = 0.0;
          marker.color.b = 0.1;
          marker.color.a = 0.5;

          // Set center position of marker as transformd mean of leaf
          marker.pose.position.x = mean(0);
          marker.pose.position.y = mean(1);
          marker.pose.position.z = mean(2);

          // Set a transformed Orientation of eigen vectors
          Eigen::Quaterniond q(eigen_vectors);
          q.normalize();
          marker.pose.orientation.x = q.x();
          marker.pose.orientation.y = q.y();
          marker.pose.orientation.z = q.z();
          marker.pose.orientation.w = q.w();

          // Set a scale of marker

          marker.scale.x = std::sqrt(eigen_values(0, 0)) * 2.2 ;
          marker.scale.y = std::sqrt(eigen_values(1, 1)) * 2.2 ;
          marker.scale.z = std::sqrt(eigen_values(2, 2)) * 2.2 ; // scaling factor: 3.0 is scaling factor for visualization 3sigma
          
          // Set a direction of marker
          marker_arrow.id = marker_id++; 
          marker_arrow.lifetime = ros::Duration(0.0);
          marker_arrow.frame_locked = true;

          
          // Set center position of marker as transformd mean of leaf
          marker_arrow.pose.position.x = mean(0);
          marker_arrow.pose.position.y = mean(1);
          marker_arrow.pose.position.z = mean(2);
        
          marker_arrow.pose.orientation.x = q.x();
          marker_arrow.pose.orientation.y = q.y();
          marker_arrow.pose.orientation.z = q.z();
          marker_arrow.pose.orientation.w = q.w();

          // Set a scale of marker
          marker_arrow.scale.x = marker.scale.x;
          marker_arrow.scale.y = marker.scale.y;
          marker_arrow.scale.z = marker.scale.z * 0.01 ; // for visualization for orientation

          marker_arrow.color.r = 0.5;
          marker_arrow.color.g = 0.0;
          marker_arrow.color.b = 0.1;
          marker_arrow.color.a = 0.5;

          // Add marker to marker array
          marker_array->markers.push_back(marker);
          marker_array_arrow->markers.push_back(marker_arrow);
        }
        debug_ndt_scan_arrow_marker_pub.publish(marker_array_arrow);
        debug_ndt_scan_marker_pub.publish(marker_array);
    }

    /*
     * @brief Publish whole accumulated NDT map for debugging.
     * @param cloud - map cloud pointer
     * @param leaves - NDT leaf map
     */
    void create_marker_array_ndt(pcl::PointCloud<PointT>::Ptr cloud,LeafMap leaves)
    {
        // Generate
        visualization_msgs::MarkerArrayPtr marker_array(new visualization_msgs::MarkerArray());
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time(cloud->header.stamp/1000000, (cloud->header.stamp%1000000) * 1000); // pcl timestamp to ros timestamp
        marker.ns = "ndt_map_scan";
        
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        int marker_id = 0;

        PointT minPt, maxPt;
        pcl::getMinMax3D(*cloud, minPt, maxPt);

        // Z축의 최소값과 최대값
        double min_z = minPt.z;
        double max_z = maxPt.z;

        // Set markers for each leaf
        for (const auto &leaf : leaves)
        {
          // Get a properties of Leaf
          auto mean = leaf.second.getMean();
          auto cov = leaf.second.getCov();
          auto eigen_vectors = leaf.second.getEvecs();
          auto eigen_values = leaf.second.getEvals();

          // Set properties
          marker.id = marker_id++; 
          marker.lifetime = ros::Duration(0.0);
          marker.frame_locked = true;
          // Set color as the heigth of the leaf
          float ratio = (mean(2) - min_z) / (max_z - min_z); //(height - min_height) / (max_height - min_height);
          double r, g, b;
          if (ratio < 0.5) { // 절반 이하일 경우 빨간색에서 초록색으로 변화
              r = 1.0 - 2 * ratio;
              g = 2 * ratio;
              b = 0.0;
          } else { // 절반 이상일 경우 초록색에서 파란색으로 변화
              r = 0.0;
              g = 1.0 - 2 * (ratio - 0.5);
              b = 2 * (ratio - 0.5);
          }
          marker.color.r = 0.1;
          marker.color.g = 0.0;
          marker.color.b = 0.5;
          marker.color.a = 0.5;
          
          // Set center position of marker as transformd mean of leaf
          marker.pose.position.x = mean(0);
          marker.pose.position.y = mean(1);
          marker.pose.position.z = mean(2);

          // Set a transformed Orientation of eigen vectors
          Eigen::Quaterniond q(eigen_vectors);
          q.normalize();
          marker.pose.orientation.x = q.x();
          marker.pose.orientation.y = q.y();
          marker.pose.orientation.z = q.z();
          marker.pose.orientation.w = q.w();

          // Set a scale of marker
          marker.scale.x = std::sqrt(eigen_values(0, 0)) * 2.2;
          marker.scale.y = std::sqrt(eigen_values(1, 1)) * 2.2;
          marker.scale.z = std::sqrt(eigen_values(2, 2)) * 2.2;

          // Add marker to marker array
          marker_array->markers.push_back(marker);
        }
        debug_ndt_map_marker_pub.publish(marker_array);
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
      }

      // check if we have any non null special nodes, if all are null then lets not bother.
      if (anchor_node->id() || anchor_edge->id() )
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

      std::ofstream ofs(directory + "/special_nodes.csv");
      ofs << "anchor_node " << (anchor_node == nullptr ? -1 : anchor_node->id()) << std::endl;
      ofs << "anchor_edge " << (anchor_edge == nullptr ? -1 : anchor_edge->id()) << std::endl;

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

      cloud->header.frame_id = map_frame_id;
      cloud->header.stamp = snapshot.back()->cloud->header.stamp;

      int ret = pcl::io::savePCDFileBinary(req.destination, *cloud);
      res.success = ret == 0;

      return true;
    }

  private:

    // Debug Variables
    ros::Publisher debug_loop_closer_aligned_pub;
    ros::Publisher debug_loop_closer_target_pub;
    ros::Publisher debug_loop_closer_source_pub;
    ros::Publisher debug_loop_closure_target_pose_pub;
    ros::Publisher debug_loop_closure_source_pose_pub;
    ros::Publisher debug_loop_closure_sub_map_pub;
    ros::Publisher debug_ndt_scan_marker_pub;       // scan elipsoidal marker publisher
    ros::Publisher debug_ndt_scan_arrow_marker_pub; // scan arrow marker publisher
    ros::Publisher debug_ndt_map_marker_pub;        // Whole Map ndt elipsoidal marker publisher
    tf2_ros::TransformBroadcaster debug_tf2_tf_broadcaster;

    // NDT Variables
    bool    create_scan_ndt_;
    bool    use_submap_loop_;
    LeafMap leaves_;
    Leaf    leaf_;
    float   leaf_voxel_size_;
    double  ndt_leaf_min_scale_;
    int     min_nr_;
  
    // ROS
    ros::NodeHandle nh;
    ros::NodeHandle mt_nh;
    ros::NodeHandle private_nh;
    ros::WallTimer optimization_timer;
    ros::WallTimer map_publish_timer;
    ros::WallTimer ndt_map_publish_timer;

    ros::Publisher markers_pub;

    std::string published_odom_topic;
    std::string map_frame_id;
    std::string odom_frame_id;

    // Odom2map gap transform
    std::mutex trans_odom2map_mutex;
    Eigen::Matrix4f trans_odom2map;
    ros::Publisher odom2map_pub;

    ros::Subscriber keyframe_sub_;
    ros::ServiceServer  updated_keyframe_client_;
    // Number of Vehicles
    unsigned int num_vehicle_;

    std::string points_topic;
    ros::Publisher read_until_pub;
    ros::Publisher map_points_pub;

    ros::ServiceServer load_service_server;
    ros::ServiceServer dump_service_server;
    ros::ServiceServer save_map_service_server;

    tf::TransformListener tf_listener;
    
    // keyframe queue
    std::string base_frame_id;
    std::mutex keyframe_queue_mutex;
    std::deque<KeyFrame::Ptr> keyframe_queue;
    std::vector<std::deque<KeyFrame::Ptr>> keyframe_queues;

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
    // Vector idx will be matched with vehicle, and deque will be matched with each vehicle's keyframe
    std::vector<std::deque<KeyFrame::Ptr>> mvdKFs; 

    g2o::VertexSE3 *anchor_node;
    g2o::EdgeSE3 *anchor_edge;
    std::vector<KeyFrame::Ptr> keyframes;

    std::unique_ptr<GraphSLAM> graph_slam;
    std::unique_ptr<LoopDetector> loop_detector;
    std::unique_ptr<KeyframeUpdater> keyframe_updater;
    std::unique_ptr<InformationMatrixCalculator> inf_calclator;
  
  }; // class HdlGraphSlamNodelet
} // namespace hdl_graph_slam

PLUGINLIB_EXPORT_CLASS(hdl_graph_slam::HdlGraphSlamNodelet, nodelet::Nodelet)
