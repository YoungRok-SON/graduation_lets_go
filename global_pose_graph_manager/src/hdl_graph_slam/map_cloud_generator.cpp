// SPDX-License-Identifier: BSD-2-Clause

#include <hdl_graph_slam/map_cloud_generator.hpp>

#include <pcl/octree/octree_search.h>

namespace hdl_graph_slam {

MapCloudGenerator::MapCloudGenerator() {}

MapCloudGenerator::~MapCloudGenerator() {}

pcl::PointCloud<MapCloudGenerator::PointT>::Ptr MapCloudGenerator::generate(const std::vector<KeyFrameSnapshot::Ptr>& keyframes, double resolution) const 
{
  if(keyframes.empty()) 
  {
    std::cerr << "warning: keyframes empty!!" << std::endl;
    return nullptr;
  }

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  cloud->reserve(keyframes.front()->cloud->size() * keyframes.size()); // 전체 포인트 개수

  for(const auto& keyframe : keyframes) 
  {
    Eigen::Matrix4f pose = keyframe->pose.matrix().cast<float>(); // 최적화된 pose를 keyframe으로부터 받아옴
    for(const auto& src_pt : keyframe->cloud->points) 
    {
      PointT dst_pt;
      dst_pt.getVector4fMap() = pose * src_pt.getVector4fMap(); // 포인트를 하나하나 다 최적화된 위치 기반으로 옮겨줌
      dst_pt.intensity = src_pt.intensity;
      cloud->push_back(dst_pt);
    }
  }

  cloud->width = cloud->size();
  cloud->height = 1;
  cloud->is_dense = false;

  if (resolution <=0.0)
    return cloud; // To get unfiltered point cloud with intensity

  pcl::octree::OctreePointCloud<PointT> octree(resolution);
  octree.setInputCloud(cloud);
  octree.addPointsFromInputCloud();

  pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
  octree.getOccupiedVoxelCenters(filtered->points);

  filtered->width = filtered->size();
  filtered->height = 1;
  filtered->is_dense = false;

  return filtered;
}

pcl::PointCloud<MapCloudGenerator::PointT>::Ptr MapCloudGenerator::generate(const std::vector<KeyFrame::Ptr>& keyframes, double resolution) const 
{
  if(keyframes.empty()) 
  {
    std::cerr << "warning: keyframes empty!!" << std::endl;
    return nullptr;
  }

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  cloud->reserve(keyframes.front()->cloud->size() * keyframes.size()); // 전체 포인트 개수

  for(const auto& keyframe : keyframes) 
  {
    Eigen::Matrix4f pose = keyframe->node->estimate().matrix().cast<float>(); // 최적화된 pose를 keyframe으로부터 받아옴
    for(const auto& src_pt : keyframe->cloud->points) 
    {
      PointT dst_pt;
      dst_pt.getVector4fMap() = pose * src_pt.getVector4fMap(); // 포인트를 하나하나 다 최적화된 위치 기반으로 옮겨줌
      dst_pt.intensity = src_pt.intensity;
      cloud->push_back(dst_pt);
    }
  }

  cloud->width = cloud->size();
  cloud->height = 1;
  cloud->is_dense = false;

  if (resolution <=0.0)
    return cloud; // To get unfiltered point cloud with intensity

  pcl::octree::OctreePointCloud<PointT> octree(resolution);
  octree.setInputCloud(cloud);
  octree.addPointsFromInputCloud();

  pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
  octree.getOccupiedVoxelCenters(filtered->points);

  filtered->width = filtered->size();
  filtered->height = 1;
  filtered->is_dense = false;

  return filtered;
}

}  // namespace hdl_graph_slam
