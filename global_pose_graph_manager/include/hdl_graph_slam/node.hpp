// SPDX-License-Identifier: BSD-2-Clause

#include <ctime>
#include <mutex>
#include <atomic>
#include <memory>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <thread>
#include <chrono>

#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl_ros/point_cloud.h>


#include <ros/ros.h>
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

namespace hdl_graph_slam {

class HdlGraphSlamNode  {


 public:
    typedef pcl::PointXYZI PointT;
    typedef pcl::PointXYZRGB PointC;
    typedef pclomp::VoxelGridCovariance<PointT>::Leaf Leaf;
    typedef std::map<size_t, pclomp::VoxelGridCovariance<PointT>::Leaf> LeafMap;

    HdlGraphSlamNode(ros::NodeHandle nh);
     ~HdlGraphSlamNode();

  bool onInit();

  private:

    void keyframe_callback(const keyframe_msgs::keyframe &keyframe_msg);

    // To Do - update pose of all keyframe DB
    bool updated_keyframe_callback(keyframe_msgs::updatedKeyFrame::Request &req, keyframe_msgs::updatedKeyFrame::Response &res);

    /**
     * @brief generate map point cloud and publish it
     * @param event
     */
    void map_points_publish_timer_callback();

    /**
     * @brief this methods adds all the data in the queues to the pose graph, and then optimizes the pose graph
     * @param event
     */
    void optimization_timer_callback();

    /**
     * @brief this method adds all the keyframes in #keyframe_queue to the pose graph (odometry edges)
     * @return if true, at least one keyframe was added to the pose graph
     */
    bool flush_keyframe_queue();


    bool manage_multi_vehicle_keyframe(int vehicle_num, Eigen::Isometry3d odom2map);

    /**
     * @brief Publish loop closre pose with point cloud
     * @param loop - loop closure data
     */
    void debug_loop_closure_points_pose(Loop::Ptr loop);
    
    /**
     * @brief create visualization marker
     * @param stamp
     * @return
     */
    visualization_msgs::MarkerArray create_marker_array(const ros::Time &stamp) const;

    /**
     * @brief create visualization marker of SINGLE keyframe for NDT Leafs
     * @param keyframes - keyframes to visualize as markers
     * @return visualization_msgs::MarkerArray
     */
    void create_marker_array_ndt(KeyFrame::Ptr keyframe);

    /*
     * @brief Publish whole accumulated NDT map for debugging.
     * @param cloud - map cloud pointer
     * @param leaves - NDT leaf map
     */
    void create_marker_array_ndt(pcl::PointCloud<PointT>::Ptr cloud,LeafMap leaves);

    /**
     * @brief save map data as pcd
     * @param req
     * @param res
     * @return
     */
    bool save_map_service(hdl_graph_slam::SaveMapRequest &req, hdl_graph_slam::SaveMapResponse &res);

    void PublishKeyFramePose( );

   

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
    ros::Publisher all_keyframe_pose_publisher_;

    // NDT Variables
    bool    create_scan_ndt_;
    bool    use_submap_loop_;
    LeafMap leaves_;
    Leaf    leaf_;
    float   leaf_voxel_size_;
    double  ndt_leaf_min_scale_;
    int     min_nr_;
  
    // ROS
    ros::NodeHandle nh_;

    ros::Publisher markers_pub;

    std::string published_odom_topic;
    std::string map_frame_id;
    std::string odom_frame_id;

    // Odom2map gap transform
    std::mutex trans_odom2map_mutex;
    std::vector<Eigen::Matrix4f> mv_trans_odom2map;
    std::vector<ros::Publisher> odom2map_pubs;

    ros::Subscriber keyframe_sub_;
    ros::ServiceServer  updated_keyframe_client_;
    // Number of Vehicles
    int num_vehicle_;

    std::string points_topic;
    ros::Publisher read_until_pub;
    std::vector<ros::Publisher> map_points_pubs;

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
    std::vector<std::vector<KeyFrameSnapshot::Ptr>> mvvKF_snapshots;
    std::unique_ptr<MapCloudGenerator> map_cloud_generator;
    int num_iterations;

    // graph slam
    // all the below members must be accessed after locking main_thread_mutex
    std::mutex main_thread_mutex;

    int max_keyframes_per_update;
    std::vector<std::deque<KeyFrame::Ptr>> mvdKFs_new;
    // Vector idx will be matched with vehicle, and deque will be matched with each vehicle's keyframe
    std::vector<std::deque<KeyFrame::Ptr>> mvdKFs; 

    bool anchor_established;
    g2o::VertexSE3 *anchor_node;
    g2o::EdgeSE3 *anchor_edge;
    std::vector<std::vector<KeyFrame::Ptr>> mvvKFs; // All Keyframes of All Vehicles

    std::unique_ptr<GraphSLAM> graph_slam;
    std::unique_ptr<LoopDetector> loop_detector;
    std::unique_ptr<KeyframeUpdater> keyframe_updater;
    std::unique_ptr<InformationMatrixCalculator> inf_calclator;

    // Thread Things
    std::thread* optimization_thread;
    std::thread* map_publish_thread;
    std::atomic_bool request_pause;
    int graph_update_interval;
    int map_cloud_update_interval;
};

}