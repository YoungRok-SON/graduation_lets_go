#include "hdl_graph_slam/node.hpp"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "global_pose_graph_manager");
  ros::NodeHandle nh;

  hdl_graph_slam::HdlGraphSlamNode global_pose_graph_optimizer(nh);
  global_pose_graph_optimizer.onInit();

  ros::spin();

  ros::shutdown();

  return 0;
}


namespace hdl_graph_slam
{

  HdlGraphSlamNode::HdlGraphSlamNode(ros::NodeHandle nh)
  {
    std::cout << "HdlGraphSlamNode Created." << std::endl;
    this->nh_ = nh;
    request_pause = false;
  }

  HdlGraphSlamNode::~HdlGraphSlamNode()
  {
    // need to finish thread
        
    request_pause = true;
    if (optimization_thread->joinable()) 
    {
        optimization_thread->join();
        std::cout << "Optimization Thread Finished. \n" << std::flush;
    }
    if (map_publish_thread->joinable()) 
    {
        map_publish_thread->join();
        std::cout << "Map Viewer Thread Finished. \n" << std::flush;
    }
  }

  bool HdlGraphSlamNode::onInit()
  {
    std::cout << "HdlGraphSlamNode initializing...." << std::endl;
    
    // init parameters
    map_frame_id = nh_.param<std::string>("global_pose_graph_manager/map_frame_id", "map");
    odom_frame_id = nh_.param<std::string>("global_pose_graph_manager/odom_frame_id", "odom_pgo");
    map_cloud_resolution = nh_.param<double>("global_pose_graph_manager/map_cloud_resolution", 0.1);
    num_iterations = nh_.param<int>("global_pose_graph_manager/g2o_solver_num_iterations", 1024);
    std::cout << "Map Cloud Resolution: " << map_cloud_resolution << std::endl;
    std::cout << "Pose Graph Optimization Iterations: " << num_iterations << std::endl;

    // 한 업데이트(최적화)마다 최대로 최적화할 수 있는 키 프레임
    // Q. 그럼 1번부터 10번까지 했으면 10번부터 19번까지 진행되는건가?
    max_keyframes_per_update = nh_.param<int>("global_pose_graph_manager/max_keyframes_per_update", 10);

    // node, edge 및 graph 관련 객체 초기화
    anchor_node = nullptr;
    anchor_edge = nullptr;
    anchor_established = false;
    graph_slam.reset(new GraphSLAM(nh_.param<std::string>("global_pose_graph_manager/g2o_solver_type", "lm_var")));

    // Graph SLAM Functionalities
    keyframe_updater.reset(new KeyframeUpdater(nh_));
    loop_detector.reset(new LoopDetector(nh_)); // Loop Closure 클래스 객체 생성
    map_cloud_generator.reset(new MapCloudGenerator());
    inf_calclator.reset(new InformationMatrixCalculator(nh_));

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /* Debugging Publisher for Loop Closure */
    debug_loop_closer_aligned_pub      = nh_.advertise<sensor_msgs::PointCloud2>("/hdl_graph_slam/debug/loop_closer_aligned", 1, true);
    debug_loop_closer_target_pub       = nh_.advertise<sensor_msgs::PointCloud2>("/hdl_graph_slam/debug/loop_closer_target", 1, true);
    debug_loop_closer_source_pub       = nh_.advertise<sensor_msgs::PointCloud2>("/hdl_graph_slam/debug/loop_closer_source", 1, true);
    debug_loop_closure_target_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("/hdl_graph_slam/debug/loop_closure_target_pose", 1, true);
    debug_loop_closure_source_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("/hdl_graph_slam/debug/loop_closure_source_pose", 1, true);

    /* Publisher for Debugging  NDT Things */
    // Maybe these topics can be changed to nh_
    debug_ndt_scan_arrow_marker_pub    = nh_.advertise<visualization_msgs::MarkerArray>("/hdl_graph_slam/debug/ndt_scan_arrow", 1, true);
    debug_ndt_scan_marker_pub          = nh_.advertise<visualization_msgs::MarkerArray>("/hdl_graph_slam/debug/ndt_scan", 1, true);
    debug_ndt_map_marker_pub           = nh_.advertise<visualization_msgs::MarkerArray>("/hdl_graph_slam/debug/ndt_map", 1, true);
    debug_loop_closure_sub_map_pub     = nh_.advertise<visualization_msgs::MarkerArray>("/hdl_graph_slam/debug/loop_closure_sub_map", 1, true);
    all_keyframe_pose_publisher_       = nh_.advertise<visualization_msgs::MarkerArray>("/hdl_graph_slam/debug/all_keyframe_pose_gpo", 1, true);

    /* Variables for NDT things */
    leaf_voxel_size_    = nh_.param<double>("global_pose_graph_manager/leaf_voxel_size", 0.5);
    create_scan_ndt_    = nh_.param<bool>("global_pose_graph_manager/create_scan_ndt", false);
    min_nr_             = nh_.param<int>("global_pose_graph_manager/min_nr", 10);
    ndt_leaf_min_scale_ = nh_.param<double>("global_pose_graph_manager/ndt_leaf_min_scale", 0.01);
    use_submap_loop_    = nh_.param<bool>("global_pose_graph_manager/use_submap_loop", false);

    // Keyframe Subscriber
    keyframe_sub_ = nh_.subscribe("/keyframe", 64, &HdlGraphSlamNode::keyframe_callback, this);

    // Service server for updatedKeyframe resquest -> callback function needs to be lock when keyframe's pose is updated.
    updated_keyframe_client_ = nh_.advertiseService("/updatedKeyframe", &HdlGraphSlamNode::updated_keyframe_callback, this);
    num_vehicle_  = nh_.param<int>("global_pose_graph_manager/num_vehicle", 2);
    std::cout << "Number of Vehicles: " << num_vehicle_ << std::endl;
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    // Publishers
    for (int vehicle_num = 0; vehicle_num < num_vehicle_; vehicle_num++)
    {
      std::string odom2map_topic = "/hdl_graph_slam/odom2map_" + std::to_string(vehicle_num);
      odom2map_pubs.push_back(nh_.advertise<geometry_msgs::TransformStamped>(odom2map_topic, 16));

      std::string map_points_topic = "/hdl_graph_slam/map_points_" + std::to_string(vehicle_num);
      map_points_pubs.push_back(nh_.advertise<sensor_msgs::PointCloud2>(map_points_topic, 1, true));
    }
    markers_pub = nh_.advertise<visualization_msgs::MarkerArray>("/hdl_graph_slam/markers", 16);
    
    // load_service_server = nh_.advertiseService("/hdl_graph_slam/load", &HdlGraphSlamNode::load_service, this);
    // dump_service_server = nh_.advertiseService("/hdl_graph_slam/dump", &HdlGraphSlamNode::dump_service, this);
    save_map_service_server = nh_.advertiseService("/hdl_graph_slam/save_map", &HdlGraphSlamNode::save_map_service, this);

    graph_updated = false;
    graph_update_interval = nh_.param<double>("global_pose_graph_manager/graph_update_interval", 3.0);
    map_cloud_update_interval = nh_.param<double>("global_pose_graph_manager/map_cloud_update_interval", 10.0);
          
    // init Keyframe Containers
    keyframe_queues.resize(num_vehicle_); // 기체 수만큼 큐를 벡터에 생성
    mvdKFs_new.resize(num_vehicle_);
    mvvKFs.resize(num_vehicle_);
    mv_trans_odom2map.resize(num_vehicle_);
    for (auto &odom2map_tf : mv_trans_odom2map)
      odom2map_tf.setIdentity();
    mvvKF_snapshots.resize(num_vehicle_);

    optimization_thread =new std::thread(&hdl_graph_slam::HdlGraphSlamNode::optimization_timer_callback, this);
    map_publish_thread  =new std::thread(&hdl_graph_slam::HdlGraphSlamNode::map_points_publish_timer_callback, this);

    
    std::cout << "HdlGraphSlamNode Initialization Done." << std::endl;

    return true;
  }

  void HdlGraphSlamNode::keyframe_callback(const keyframe_msgs::keyframe &keyframe_msg)
  {
    // Get Keyframe id and vehicle id 
    const int &keyframe_id       = keyframe_msg.id;
    const int &vehicle_id        = keyframe_msg.vehicle_id;

    // Get odom pose
    std::cout << "Got you Keyframe" << std::endl;
    const ros::Time &stamp       = keyframe_msg.header.stamp;
    Eigen::Isometry3d odom = pose2isometry(keyframe_msg.Pose);
    
    // Get Point Cloud Data
    pcl::PointCloud<PointC>::Ptr cloud(new pcl::PointCloud<PointC>());
    pcl::fromROSMsg(keyframe_msg.PointCloud, *cloud);
    std::string str_vehicle = std::to_string(vehicle_id);
    vehicle_camera_link_name = "vehicle_"+ str_vehicle + "/orb_slam2_rgbd/camera_link";
    vehicle_base_link_name = "vehicle_"+ str_vehicle + "/orb_slam2_rgbd/base_link";
    vehicle_camera_color_optical_frame_name = "vehicle_"+ str_vehicle + + "/orb_slam2_rgbd/camera_color_optical_frame";

    tf::StampedTransform transform;
    if (!tf_listener.canTransform(vehicle_base_link_name, vehicle_camera_color_optical_frame_name, ros::Time(0))) // IMU에 대한 tf 검사도 해야한는 거 아닌가/
    {
      std::cerr << "failed to find transform between " << "base_link" << " and " << "camera_color_optical_frame" << std::endl;
    }
    tf_listener.waitForTransform(vehicle_camera_link_name, vehicle_camera_color_optical_frame_name, ros::Time(0), ros::Duration(2.0));
    tf_listener.lookupTransform(vehicle_camera_link_name, vehicle_camera_color_optical_frame_name, ros::Time(0), transform);

    pcl::PointCloud<PointC>::Ptr transformed(new pcl::PointCloud<PointC>());
    pcl_ros::transformPointCloud(*cloud, *transformed, transform);
    transformed->header.frame_id = vehicle_base_link_name;
    

    // 이전 keyframe과의 거리 계산을 통해 짧으면 false, 길면 true
    if (!keyframe_updater->update(odom)) 
      return;

    // Create Keyframe object
    double accum_d = keyframe_updater->get_accum_distance();
    KeyFrame::Ptr keyframe = KeyFrame::Ptr(new KeyFrame(stamp, odom, accum_d, transformed, keyframe_id, vehicle_id));

    std::lock_guard<std::mutex> lock(keyframe_queue_mutex); // 뮤텍스 걸고
    keyframe_queues[keyframe->vehicle_id].push_back(keyframe); // 생성한 키프레임 객체를 id에 맞는 큐에 넣기
  }

  // To Do - update pose of all keyframe DB
  bool HdlGraphSlamNode::updated_keyframe_callback(keyframe_msgs::updatedKeyFrame::Request &req, keyframe_msgs::updatedKeyFrame::Response &res)
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

  void HdlGraphSlamNode::optimization_timer_callback()
  {
    while(1)
    {
      if(request_pause)
        break;
      std::this_thread::sleep_for( std::chrono::seconds(graph_update_interval) );

      std::cout << "Optization Thread Running... \n" << std::flush;
      std::lock_guard<std::mutex> lock(main_thread_mutex);

      // add keyframes in the queues to the pose graph
      bool keyframe_updated = flush_keyframe_queue(); // for local optimization
      for(int vehicle_num = 0; vehicle_num < num_vehicle_; vehicle_num++)
      {
        if (mvvKFs[vehicle_num].empty())
          continue;
        std::cout << "Vehicle Num: " << vehicle_num << "Updated Node: " << mvvKFs[vehicle_num].size() << "\n" << std::flush;
      }

      if (!keyframe_updated)
        continue;

      // loop detection 
      // To Do 
      // Detect Loop Closing in case of multi-vehicle
      // 1. N:N Matching 
      // 2. N:M Matching
      std::cout << "Loop Detection Started.\n" << std::flush;
      for (size_t vehicle_idx = 0; vehicle_idx < num_vehicle_; vehicle_idx++)
      {
        for (size_t target_vehicle_idx = 0; target_vehicle_idx < num_vehicle_; target_vehicle_idx++)
        {

          std::vector<Loop::Ptr> loops = loop_detector->detect(mvvKFs[target_vehicle_idx], mvdKFs_new[vehicle_idx], *graph_slam);
          // 새로운 키프래임과 이전 키프레임 사이의 매칭 진행? -> N(기존 키프레임들) : M(새로운 키프레임들) 사이의 매칭을 진행해서 여러개가 나올 수 있음.
          for (const auto &loop : loops)
          {
            Eigen::Isometry3d relpose(loop->relative_pose.cast<double>());
            Eigen::MatrixXd information_matrix = inf_calclator->calc_information_matrix(loop->key1->cloud_t, loop->key2->cloud_t, relpose);
            auto edge = graph_slam->add_se3_edge(loop->key1->node, loop->key2->node, relpose, information_matrix);
            
            graph_slam->add_robust_kernel(edge, nh_.param<std::string>("global_pose_graph_manager/loop_closure_edge_robust_kernel", "NONE"), nh_.param<double>("global_pose_graph_manager/loop_closure_edge_robust_kernel_size", 1.0));
            // Debug Publisher    
            if(debug_loop_closer_target_pub.getNumSubscribers() && debug_loop_closer_source_pub.getNumSubscribers())
              debug_loop_closure_points_pose(loop);
          }
        }
        std::copy(mvdKFs_new[vehicle_idx].begin(), mvdKFs_new[vehicle_idx].end(), std::back_inserter(mvvKFs[vehicle_idx]));
        // new_keyframes의 모든 원소들이 keyframes의 끝에 추가
        mvdKFs_new[vehicle_idx].clear();
      }
      std::cout << "Loop Detection Done. \n " << keyframe_updated << std::flush;
      // optimize the pose graph
    
      graph_slam->optimize(num_iterations); // 지정한 회수만큼 최적화 진행
      graph_updated = true;

      std::cout << "Graph Optimization Done. "  << "\n" << std::flush;

      // publish tf
      trans_odom2map_mutex.lock();
      for (int vehicle_num = 0; vehicle_num < num_vehicle_; vehicle_num++)
      {
        if(mvvKFs[vehicle_num].empty())
          continue;
        const auto &keyframe = mvvKFs[vehicle_num].back();
        Eigen::Isometry3d trans = keyframe->node->estimate() * keyframe->odom.inverse(); // 이건 어떤 값? -> 오도메트리와 계산된 그래프를 기반으로 최적화된 위치 결과 사이의 변환, 결국 누적 오차 없에는 역할
        mv_trans_odom2map[vehicle_num] = trans.matrix().cast<float>(); // 이 값은 왜 이렇게 해놓지?-> 다음  keyframe이 들어올 때 이전 keyframe과의 차이를 계산하기 위해서
      }
      trans_odom2map_mutex.unlock();
      std::cout << " trans_odom2map Done " << "\n" << std::flush;

      // 벡터인 Keyframes에 있는 객체들을 복사해서 snapshot이라는 벡터 컨테이너에 KeyFrameSnapshot이라는 형태로 저장
      // 결국 새로운 키프래임들을 같이 하나의 맵 데이터로 저장
      for (int vehicle_num = 0; vehicle_num < num_vehicle_; vehicle_num++)
      {
        std::vector<KeyFrameSnapshot::Ptr> snapshot(mvvKFs[vehicle_num].size());
        std::transform(mvvKFs[vehicle_num].begin(), mvvKFs[vehicle_num].end(), snapshot.begin(), [=](const KeyFrame::Ptr &k)
                      { return std::make_shared<KeyFrameSnapshot>(k); });

        keyframes_snapshot_mutex.lock();
        mvvKF_snapshots[vehicle_num].swap(snapshot);
        keyframes_snapshot_mutex.unlock();
      }
      std::cout << " Keyframe Snapshot Done " << "\n" << std::flush;

      // publish odom and markers topics of all vehicles.
      for ( int vehicle_idx = 0; vehicle_idx < num_vehicle_; vehicle_idx++ )
      {
        if (odom2map_pubs[vehicle_idx].getNumSubscribers())
        {
          geometry_msgs::TransformStamped ts = matrix2transform(mvvKFs[vehicle_idx].back()->stamp, mv_trans_odom2map[vehicle_idx], map_frame_id, odom_frame_id);
          odom2map_pubs[vehicle_idx].publish(ts);
        }
      }

      if (markers_pub.getNumSubscribers())
      {
        auto markers = create_marker_array(ros::Time::now());
        markers_pub.publish(markers);
      }
      std::cout << "Optization Thread Done. Wating a time..." << std::flush;
    }
  }




  bool HdlGraphSlamNode::flush_keyframe_queue()
  {
    std::vector<Eigen::Isometry3d> vec_odom2map;
    trans_odom2map_mutex.lock();
    for (auto &trans_odom2map : mv_trans_odom2map)
    {
      Eigen::Isometry3d odom2map(trans_odom2map.cast<double>()); // trans_odom2map 변수 값을 double로 캐스팅해서 odom2map이라는 변수 생성
      vec_odom2map.push_back(odom2map);
    }
    trans_odom2map_mutex.unlock();

    // check whether the queue is empty
    bool element_exist = false;
    std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
    for (int vehicle_idx = 0; vehicle_idx < keyframe_queues.size(); vehicle_idx++)
    {
      
      if (!keyframe_queues[vehicle_idx].empty())
      {
        int num_processed = 0;
        // element_exist = manage_multi_vehicle_keyframe(vehicle_idx, vec_odom2map[vehicle_idx]);
        for (int i = 0; i < std::min<int>(keyframe_queues[vehicle_idx].size(), max_keyframes_per_update); i++) // Keyframe queue에 있는 개수가 min보다 작으면 개수만큼만 진행
        { 
          num_processed = i; // processed keyframe 개수

          const auto &keyframe = keyframe_queues[vehicle_idx][i];
          // new_keyframes will be tested later for loop closure
          mvdKFs_new[vehicle_idx].push_back(keyframe);

          // add pose node
          Eigen::Isometry3d odom = vec_odom2map[vehicle_idx] * keyframe->odom;
          // Keyframe 객체를 생성할 때 노드에 대한 값이 비어 있음. 그걸 이때 채워넣음. 그리고 그래프 안에도 넣어줌
          keyframe->node = graph_slam->add_se3_node(odom); 
          
          // fix the first node
          if (mvvKFs[vehicle_idx].empty() && mvdKFs_new[vehicle_idx].size() == 1) // check each vehicle's first keyframe
          {

            if (nh_.param<bool>("global_pose_graph_manager/fix_first_node", false))
            {
              Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
              if ( !anchor_established )
              {
                std::stringstream sst(nh_.param<std::string>("fix_first_nglobal_pose_graph_manager/fix_first_node_stddev", "1 1 1 1 1 1"));
                for (int i = 0; i < 6; i++)
                {
                  double stddev = 1.0;
                  sst >> stddev;
                  inf(i, i) = 1.0 / stddev;
                }

                anchor_node = graph_slam->add_se3_node(Eigen::Isometry3d::Identity()); 
                anchor_node->setFixed(true);  // 고정
                anchor_established = true;
              }
              anchor_edge = graph_slam->add_se3_edge(anchor_node, keyframe->node, Eigen::Isometry3d::Identity(), inf); // 앵커노드와 각 vehicle의 첫번째 키프레임 사이 에지를 지정
              std::cout << "Anchor Vertex Fixed. \n" << std::endl;
            }
          }

          if (i == 0 && mvvKFs[vehicle_idx].empty()) // 만약 Keyframe들을 모아놓는 벡터에 아무것도 없으면 그냥 넘어감
          {

            continue; // Anchor Node와 첫 키프레임 사이의 에지를 추가하고 넘어감
          }

          /* --------------------------------------------------------------------------- */
          // add edge between consecutive keyframes
          const auto &prev_keyframe = i == 0 ? mvvKFs[vehicle_idx].back() : keyframe_queues[vehicle_idx][i - 1]; // 일단 i==0 일때 현재 키프레임과 이전 키프레임을 엮어줘야하니 prev_keyframe을 지정해주는 것으로 보임
          /* --------------------------------------------------------------------------- */

          std::cout << "KeyFrame Added to Graph. \n" << std::endl;
          // 연결된 두 포즈 사이의 상대 포즈를 계산
          Eigen::Isometry3d relative_pose = keyframe->odom.inverse() * prev_keyframe->odom; // 이전 키프레임 -> 현재 키프레임으로 가는 상대 포즈

          // 연결된 두 포주 사이의 정보 행렬 계산 -> 고정된 값으로 진행(ORB), 조금 더 큰 값(NDT), 아루코마커 위치 정보에 대한 정보 행렬도 지정해서 그냥 사용
          static double stddev_x = nh_.param<double>("global_pose_graph_manager/const_stddev_x", 0.0l);
          static double stddev_q = nh_.param<double>("global_pose_graph_manager/const_stddev_q", 0.1);
          Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
          inf.topLeftCorner(3, 3).array() /= stddev_x*stddev_x;     // need to be increased by Uncertainty increase
          inf.bottomRightCorner(3, 3).array() /= stddev_q*stddev_q; // need to be increased by Uncertainty increase
          std::cout << "Information Matrix Calculated. " << inf << std::endl;

          // 연결된 두 포즈 사이의 에지를 추가
          auto edge = graph_slam->add_se3_edge(keyframe->node, prev_keyframe->node, relative_pose, inf);
          // Robust Kernel 추가
          graph_slam->add_robust_kernel(edge, nh_.param<std::string>("global_pose_graph_manager/odometry_edge_robust_kernel", "NONE"), nh_.param<double>("global_pose_graph_manager/odometry_edge_robust_kernel_size", 1.0));
          element_exist = true;
        }
        // 3초에 한번씩 도니까 한번 싹 밀어 넣은 다음에 처리한건 다 지움
        keyframe_queues[vehicle_idx].erase( keyframe_queues[vehicle_idx].begin(),  keyframe_queues[vehicle_idx].begin() + num_processed + 1);
      }
    }
    return element_exist;
  }








  void HdlGraphSlamNode::map_points_publish_timer_callback()
  {
    while(1)
    {
      if(request_pause)
        break;

      std::this_thread::sleep_for( std::chrono::seconds(map_cloud_update_interval) );

      std::cout << "Map Point Publish Thread is Running...\n " << std::flush;
      for (int vehicle_num = 0; vehicle_num < num_vehicle_; vehicle_num++)
      {
        if ( !map_points_pubs[vehicle_num].getNumSubscribers() || !graph_updated )
        {
          std::cout << "Map Point Publisher Denied... \n" << std::endl;
          continue;
        }
        
        std::cout << "Generate Map Cloud Vehicle Number: " + std::to_string(vehicle_num)  << "\n"<< std::endl;
        std::vector<KeyFrameSnapshot::Ptr> snapshot;

        keyframes_snapshot_mutex.lock();
        snapshot = mvvKF_snapshots[vehicle_num]; // 처음 키프레임 ~ 현제까지 flush된 키프레임 집합
        keyframes_snapshot_mutex.unlock();

        auto cloud = map_cloud_generator->generate(snapshot, map_cloud_resolution);

        if (!cloud)
        {
          continue;;
        }

        cloud->header.frame_id = map_frame_id;
        cloud->header.stamp = snapshot.back()->cloud->header.stamp;

        ROS_INFO("Map Size: %ld", cloud->size());

        sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2()); 
        pcl::toROSMsg(*cloud, *cloud_msg);  // 실행하면 cloud가 비워짐.
        map_points_pubs[vehicle_num].publish(cloud_msg);
        PublishKeyFramePose();
        std::cout << "Map Point Publish Thread Done. Wating a time... \n" << std::endl;
      }
    }
  }





  void HdlGraphSlamNode::debug_loop_closure_points_pose(Loop::Ptr loop)
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

  visualization_msgs::MarkerArray HdlGraphSlamNode::create_marker_array(const ros::Time &stamp) const
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

    
    traj_marker.points.resize(graph_slam->num_vertices());
    traj_marker.colors.resize(graph_slam->num_vertices());
    for (int i = 0; i < graph_slam->num_vertices(); i++)
    {
      g2o::VertexSE3 *v = dynamic_cast<g2o::VertexSE3 *>(graph_slam->graph->vertices()[i]);
      Eigen::Vector3d pos = v->estimate().translation();
      traj_marker.points[i].x = pos.x();
      traj_marker.points[i].y = pos.y();
      traj_marker.points[i].z = pos.z();

      double p = static_cast<double>(i) / graph_slam->num_vertices();
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
    }

    // sphere
    visualization_msgs::Marker &sphere_marker = markers.markers[3];
    sphere_marker.header.frame_id = "map";
    sphere_marker.header.stamp = stamp;
    sphere_marker.ns = "loop_close_radius";
    sphere_marker.id = 3;
    sphere_marker.type = visualization_msgs::Marker::SPHERE;

    sphere_marker.points.resize(num_vehicle_);
    sphere_marker.colors.resize(num_vehicle_);
    for (auto keyframes : mvvKFs)
    {
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
      }
    return markers;
  }

  void HdlGraphSlamNode::create_marker_array_ndt(KeyFrame::Ptr keyframe)
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

  void HdlGraphSlamNode::create_marker_array_ndt(pcl::PointCloud<PointT>::Ptr cloud,LeafMap leaves)
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

  bool HdlGraphSlamNode::save_map_service(hdl_graph_slam::SaveMapRequest &req, hdl_graph_slam::SaveMapResponse &res)
    {
      std::vector<KeyFrameSnapshot::Ptr> snapshot;

      keyframes_snapshot_mutex.lock();
      for (auto &ks : mvvKF_snapshots)
      {
        snapshot.insert(snapshot.end(), ks.begin(), ks.end());
      }
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



  void HdlGraphSlamNode::PublishKeyFramePose( ) 
  {
    if (mvvKFs[0].empty())
    {
      ROS_WARN("Keyframe vector is empty!");
      return;
    }

    
    // Generate 3 arrows for each keyframe
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    marker.header.frame_id = map_frame_id;
    marker.header.stamp = mvvKFs[0].back()->stamp;
    marker.ns = "keyframe_pose";
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.0);


    for ( auto it = mvvKFs[0].begin(); it != mvvKFs[0].end(); it++) 
    {

      // Get the keyframe
      KeyFrame::Ptr pKF = *it;

      // Get pose
      auto pose = pKF->node->estimate();

      // Iso3d -> tf2
      Eigen::Vector3d position_vec_eigen = pose.translation();
      Eigen::Quaterniond orientation_quat_eigen(pose.rotation());
      tf2::Vector3 position_vec(position_vec_eigen.x(), position_vec_eigen.y(), position_vec_eigen.z());
      tf2::Quaternion orientation_quat(orientation_quat_eigen.x(), orientation_quat_eigen.y(), orientation_quat_eigen.z(), orientation_quat_eigen.w());

      
      // Create the x, y, and z arrows
      visualization_msgs::Marker x_arrow = marker;
      x_arrow.id = (pKF)->keyframe_id * 3;
      x_arrow.pose.position.x = position_vec.x();
      x_arrow.pose.position.y = position_vec.y();
      x_arrow.pose.position.z = position_vec.z();
      x_arrow.pose.orientation.x = orientation_quat.x();
      x_arrow.pose.orientation.y = orientation_quat.y();
      x_arrow.pose.orientation.z = orientation_quat.z();
      x_arrow.pose.orientation.w = orientation_quat.w();
      x_arrow.scale.x = 0.2;
      x_arrow.scale.y = 0.01;
      x_arrow.scale.z = 0.01;
      x_arrow.color.r = 1.0;
      x_arrow.color.g = 0.0;
      x_arrow.color.b = 0.0;
      x_arrow.color.a = 1.0;

      // Create the y axis arrow by rotate the x axis arrow
      visualization_msgs::Marker y_arrow = marker;
      y_arrow.id = pKF->keyframe_id * 3 + 1;
      y_arrow.pose.position.x = position_vec.x();
      y_arrow.pose.position.y = position_vec.y();
      y_arrow.pose.position.z = position_vec.z();
      // Rotate axis by 90 degrees around z axis of x_arrow
      tf2::Quaternion y_quat;
      y_quat.setRPY(0, 0, M_PI/2);
      tf2::Quaternion y_axis = orientation_quat * y_quat;
      y_arrow.pose.orientation.x = y_axis.x();
      y_arrow.pose.orientation.y = y_axis.y();
      y_arrow.pose.orientation.z = y_axis.z();
      y_arrow.pose.orientation.w = y_axis.w();
      y_arrow.scale.x = 0.2;
      y_arrow.scale.y = 0.01;
      y_arrow.scale.z = 0.01;
      y_arrow.color.r = 0.0;
      y_arrow.color.g = 1.0;
      y_arrow.color.b = 0.0;
      y_arrow.color.a = 1.0;

      // Cretae the z axis arrow by rotating the x axis arrow
      visualization_msgs::Marker z_arrow = marker;
      z_arrow.id = pKF->keyframe_id * 3 + 2;
      z_arrow.pose.position.x = position_vec.x();
      z_arrow.pose.position.y = position_vec.y();
      z_arrow.pose.position.z = position_vec.z();
      tf2::Quaternion z_quat;
      z_quat.setRPY(0, -M_PI/2, 0);
      tf2::Quaternion z_axis = orientation_quat * z_quat;
      z_arrow.pose.orientation.x = z_axis.x();
      z_arrow.pose.orientation.y = z_axis.y();
      z_arrow.pose.orientation.z = z_axis.z();
      z_arrow.pose.orientation.w = z_axis.w();
      z_arrow.scale.x = 0.2;
      z_arrow.scale.y = 0.01;
      z_arrow.scale.z = 0.01;
      z_arrow.color.r = 0.0;
      z_arrow.color.g = 0.0;
      z_arrow.color.b = 1.0;
      z_arrow.color.a = 1.0;
      
      // Add the arrows to the marker array
      marker_array.markers.push_back(x_arrow);
      marker_array.markers.push_back(y_arrow);
      marker_array.markers.push_back(z_arrow);
    }

    // Publish the marker array
    all_keyframe_pose_publisher_.publish(marker_array);
  }

  
} // namespace hdl_graph_slam