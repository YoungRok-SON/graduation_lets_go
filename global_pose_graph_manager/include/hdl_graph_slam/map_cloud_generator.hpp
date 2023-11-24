// SPDX-License-Identifier: BSD-2-Clause

#ifndef MAP_CLOUD_GENERATOR_HPP
#define MAP_CLOUD_GENERATOR_HPP

#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <hdl_graph_slam/keyframe.hpp>
#include <g2o/types/slam3d/vertex_se3.h>


namespace hdl_graph_slam {

/**
 * @brief this class generates a map point cloud from registered keyframes
 */
class MapCloudGenerator {
public:
  using PointT = pcl::PointXYZI;
  using PointC = pcl::PointXYZRGB;

  MapCloudGenerator();
  ~MapCloudGenerator();

  /**
   * @brief generates a map point cloud
   * @param keyframes   snapshots of keyframes
   * @param resolution  resolution of generated map
   * @return generated map point cloud
   */
  pcl::PointCloud<PointC>::Ptr generate(const std::vector<KeyFrameSnapshot::Ptr>& keyframes, double resolution) const;
  pcl::PointCloud<PointC>::Ptr generate(const std::vector<KeyFrame::Ptr>& keyframes, double resolution) const;
  pcl::PointCloud<PointT>::Ptr generateXYZICloud(const std::vector<KeyFrame::Ptr>& keyframes, double resolution) const ;

};

}  // namespace hdl_graph_slam

#endif  // MAP_POINTCLOUD_GENERATOR_HPP