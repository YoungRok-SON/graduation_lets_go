// SPDX-License-Identifier: BSD-2-Clause

#include <hdl_graph_slam/map_cloud_generator.hpp>

#include <pcl/octree/octree_search.h>

namespace hdl_graph_slam {

MapCloudGenerator::MapCloudGenerator() {}

MapCloudGenerator::~MapCloudGenerator() {}

pcl::PointCloud<MapCloudGenerator::PointC>::Ptr MapCloudGenerator::generate(const std::vector<KeyFrameSnapshot::Ptr>& keyframes, double resolution) const 
{
  if(keyframes.empty()) 
  {
    std::cerr << "warning: keyframes empty!!" << std::endl;
    return nullptr;
  }

  pcl::PointCloud<PointC>::Ptr cloud(new pcl::PointCloud<PointC>());
  cloud->reserve(keyframes.front()->cloud->size() * keyframes.size()); // 전체 포인트 개수

  for(const auto& keyframe : keyframes) 
  {
    Eigen::Matrix4f pose = keyframe->pose.matrix().cast<float>(); // 최적화된 pose를 keyframe으로부터 받아옴
    for(const auto& src_pt : keyframe->cloud->points) 
    {
      PointC dst_pt;
      dst_pt.getVector4fMap() = pose * src_pt.getVector4fMap(); // 포인트를 하나하나 다 최적화된 위치 기반으로 옮겨줌
      dst_pt.r = src_pt.r;
      dst_pt.g = src_pt.g;
      dst_pt.b = src_pt.b;
      cloud->push_back(dst_pt);
    }
  }

  cloud->width = cloud->size();
  cloud->height = 1;
  cloud->is_dense = false;

  if (resolution <=0.0)
    return cloud; // To get unfiltered point cloud with intensity

  pcl::octree::OctreePointCloud<PointC> octree(resolution);
  octree.setInputCloud(cloud);
  octree.addPointsFromInputCloud();

  pcl::PointCloud<PointC>::Ptr filtered(new pcl::PointCloud<PointC>());
  octree.getOccupiedVoxelCenters(filtered->points);

  filtered->width = filtered->size();
  filtered->height = 1;
  filtered->is_dense = false;

  return filtered;
}

pcl::PointCloud<MapCloudGenerator::PointC>::Ptr MapCloudGenerator::generate(const std::vector<KeyFrame::Ptr>& keyframes, double resolution) const 
{
  if(keyframes.empty()) 
  {
    std::cerr << "warning: keyframes empty!!" << std::endl;
    return nullptr;
  }

  pcl::PointCloud<PointC>::Ptr cloud(new pcl::PointCloud<PointC>());
  cloud->reserve(keyframes.front()->cloud_c->size() * keyframes.size()); // 전체 포인트 개수

  for(const auto& keyframe : keyframes) 
  {
    Eigen::Matrix4f pose = keyframe->node->estimate().matrix().cast<float>(); // 최적화된 pose를 keyframe으로부터 받아옴
    for(const auto& src_pt : keyframe->cloud_c->points) 
    {
      PointC dst_pt;
      dst_pt.getVector4fMap() = pose * src_pt.getVector4fMap(); // 포인트를 하나하나 다 최적화된 위치 기반으로 옮겨줌
      dst_pt.r = src_pt.r;
      dst_pt.g = src_pt.g;
      dst_pt.b = src_pt.b;
      cloud->push_back(dst_pt);
    }
  }

  cloud->width = cloud->size();
  cloud->height = 1;
  cloud->is_dense = false;

  if (resolution <=0.0)
    return cloud; // To get unfiltered point cloud with intensity

  pcl::octree::OctreePointCloud<PointC> octree(resolution);
  octree.setInputCloud(cloud);
  octree.addPointsFromInputCloud();

  pcl::PointCloud<PointC>::Ptr filtered(new pcl::PointCloud<PointC>());
  octree.getOccupiedVoxelCenters(filtered->points);

  filtered->width = filtered->size();
  filtered->height = 1;
  filtered->is_dense = false;

  return filtered;
}

pcl::PointCloud<MapCloudGenerator::PointT>::Ptr MapCloudGenerator::generateXYZICloud(const std::vector<KeyFrame::Ptr>& keyframes, double resolution) const 
{
  if(keyframes.empty()) 
  {
    std::cerr << "warning: keyframes empty!!" << std::endl;
    return nullptr;
  }

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  cloud->reserve(keyframes.front()->cloud_t->size() * keyframes.size()); // 전체 포인트 개수

  for(const auto& keyframe : keyframes) 
  {
    Eigen::Matrix4f pose = keyframe->node->estimate().matrix().cast<float>(); // 최적화된 pose를 keyframe으로부터 받아옴
    for(const auto& src_pt : keyframe->cloud_t->points) 
    {
      PointT dst_pt;
      dst_pt.getVector4fMap() = pose * src_pt.getVector4fMap(); // 포인트를 하나하나 다 최적화된 위치 기반으로 옮겨줌
      dst_pt.intensity = 0.0;
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
