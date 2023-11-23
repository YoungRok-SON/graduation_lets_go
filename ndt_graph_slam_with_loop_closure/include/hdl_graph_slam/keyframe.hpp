// SPDX-License-Identifier: BSD-2-Clause

#ifndef KEYFRAME_HPP
#define KEYFRAME_HPP

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/optional.hpp>

// For New Featuers
#include <pclomp/ndt_omp.h>
#include <pclomp/voxel_grid_covariance_omp.h>

/* 
KeyFrame은 로봇이 환경에서 얻은 원시 데이터와 관련 메타데이터를 저장
KeyFrameSnapshot은 SLAM 처리 후의 최적화된 결과를 저장
따라서 두 구조체는 서로 다른 상황과 목적으로 사용될 수 있습니다. 

Q. 최적화된 키프레임을 따로 저장한다면, 그래프 자체에 저장을 하는건 아닌건가?
그래프에 있는 노드는 업데이트를 어떻게 하는거지??
*/

namespace g2o
{
  class VertexSE3;
  class HyperGraph;
  class SparseOptimizer;
} // namespace g2o

namespace hdl_graph_slam
{

  /**
   * @brief KeyFrame (pose node)
   */
  struct KeyFrame // 아래 KeyFrameSnapshot이랑 차이점이..
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using PointT = pcl::PointXYZI;
    using PointC = pcl::PointXYZRGB;
    using Ptr = std::shared_ptr<KeyFrame>;
    typedef pclomp::VoxelGridCovariance<PointT>::Leaf Leaf;
    typedef std::map<size_t, pclomp::VoxelGridCovariance<PointT>::Leaf> LeafMap;
    pclomp::VoxelGridCovariance<PointT> generate_ndt_scan;


    KeyFrame(const ros::Time &stamp, const Eigen::Isometry3d &odom, double accum_distance, const pcl::PointCloud<PointT>::Ptr &cloud);
    KeyFrame(const ros::Time &stamp, const Eigen::Isometry3d &odom, double accum_distance, const pcl::PointCloud<PointC>::Ptr &cloud, int keyframe_id, int vehicle_id);
    KeyFrame(const ros::Time &stamp, const Eigen::Isometry3d &odom, double accum_distance, const pcl::PointCloud<PointT>::Ptr &cloud, float leaf_size, int min_nr);
    KeyFrame(const std::string &directory, g2o::HyperGraph *graph);
    virtual ~KeyFrame();

    void save(const std::string &directory);
    bool load(const std::string &directory, g2o::HyperGraph *graph);

    long id() const;
    Eigen::Isometry3d estimate() const;

  public:
    ros::Time stamp;                               // timestamp
    Eigen::Isometry3d odom;                        // odometry (estimated by scan_matching_odometry)
    double accum_distance;                         // accumulated distance from the first node (by scan_matching_odometry)
    pcl::PointCloud<PointC>::Ptr cloud_c;       // point cloud color
    pcl::PointCloud<PointT>::Ptr cloud_t;       // point cloud intensity
    
    int keyframe_id;                                // keyframe id
    int vehicle_id;                                 // vehicle id
    // vertex 번호도 필요한가..? node에서 가져올 수 있지 않을까?

    g2o::VertexSE3 *node; // node instance

    // For new features
    LeafMap leaves;
    float   leaf_size;
    int     min_nr;
  };




  /**
   * @brief KeyFramesnapshot for map cloud generation
   */
  struct KeyFrameSnapshot  // 아마 키프레임에 대한 포인트클라우드를 저장하는 용도인 듯 함
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using PointT = KeyFrame::PointT;
    using PointC = KeyFrame::PointC;
    using Ptr = std::shared_ptr<KeyFrameSnapshot>;
    typedef pclomp::VoxelGridCovariance<PointT>::Leaf Leaf;
    typedef std::map<size_t, pclomp::VoxelGridCovariance<PointT>::Leaf> LeafMap;

    KeyFrameSnapshot(const KeyFrame::Ptr &key);
    KeyFrameSnapshot(const Eigen::Isometry3d &pose, const pcl::PointCloud<PointC>::ConstPtr &cloud_c);

    ~KeyFrameSnapshot();

  public:
    Eigen::Isometry3d pose;                  // pose estimated by graph optimization
    pcl::PointCloud<PointC>::ConstPtr cloud; // point cloud
    // For new features
    LeafMap leaves;
  };

} // namespace hdl_graph_slam

#endif // KEYFRAME_HPP