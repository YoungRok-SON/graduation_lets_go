// SPDX-License-Identifier: BSD-2-Clause

#include <hdl_graph_slam/keyframe.hpp>

#include <boost/filesystem.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/vertex_se3.h>

namespace hdl_graph_slam
{
  typedef pcl::PointXYZRGB PointC;
  typedef pcl::PointXYZI PointT;


  KeyFrame::KeyFrame( const ros::Time &stamp, const Eigen::Isometry3d &odom, 
                      double accum_distance, 
                      const pcl::PointCloud<PointT>::Ptr &cloud) : 
                      stamp(stamp),
                      odom(odom),
                      accum_distance(accum_distance), 
                      cloud_t(cloud), 
                      node(nullptr) 
  {
  }

  KeyFrame::KeyFrame( const ros::Time &stamp,
                      const Eigen::Isometry3d &odom, 
                      double accum_distance, 
                      const pcl::PointCloud<PointT>::Ptr &cloud,
                      float leaf_size,
                      int min_nr) :
                      stamp(stamp),
                      odom(odom),
                      accum_distance(accum_distance),
                      cloud_t(cloud), 
                      node(nullptr),
                      leaf_size(leaf_size),
                      min_nr(min_nr)
  {
    generate_ndt_scan.setInputCloud(cloud);
    generate_ndt_scan.setLeafSize(leaf_size, leaf_size, leaf_size);
    generate_ndt_scan.setMinimumPointsNumberPerVoxel(min_nr);
    generate_ndt_scan.filter();  
    leaves = generate_ndt_scan.getLeaves();
  }

  KeyFrame::KeyFrame( const ros::Time &stamp, const Eigen::Isometry3d &odom,
                      double accum_distance, const pcl::PointCloud<PointC>::Ptr &cloud,
                      int keyframe_id, int vehicle_id)
                      : stamp(stamp), odom(odom),
                        accum_distance(accum_distance), cloud_c(nullptr),cloud_t(nullptr),
                        node(nullptr), 
                        keyframe_id(keyframe_id), vehicle_id(vehicle_id)
  {
      this->cloud_c = cloud;
      // conver xyzrgb to xyzi
      this->cloud_t->resize(cloud->size());
      for (int i = 0; i < cloud->size(); i++)
      {
        pcl::PointXYZI dst_p;
        dst_p.x = cloud->points[i].x;
        dst_p.y = cloud->points[i].y;
        dst_p.z = cloud->points[i].z;
        dst_p.intensity = 0.0;
        this->cloud_t->points[i] = dst_p;
      }
  }


  KeyFrame::KeyFrame(const std::string &directory, g2o::HyperGraph *graph) : stamp(), odom(Eigen::Isometry3d::Identity()), accum_distance(-1), cloud_t(nullptr), node(nullptr)
  {
    load(directory, graph);
  }

  KeyFrame::~KeyFrame() {}

  void KeyFrame::save(const std::string &directory)
  {
    if (!boost::filesystem::is_directory(directory))
    {
      boost::filesystem::create_directory(directory);
    }

    std::ofstream ofs(directory + "/data");
    ofs << "stamp " << stamp.sec << " " << stamp.nsec << "\n";

    ofs << "estimate\n";
    ofs << node->estimate().matrix() << "\n";

    ofs << "odom\n";
    ofs << odom.matrix() << "\n";

    ofs << "accum_distance " << accum_distance << "\n";

    if (node)
    {
      ofs << "id " << node->id() << "\n";
    }

    pcl::io::savePCDFileBinary(directory + "/cloud.pcd", *cloud_c);
  }


  /**
  * @brief 이 함수는 저장된 KeyFrame 객체의 정보를 파일 시스템에서 불러와 다시 해당 객체로 복원하기 위해 사용됩니다.
  * @param directory 저장된 디렉토리
  * @param graph 다시 불러와 저장할 graph 객체
  * @return 성공유무
  */
  bool KeyFrame::load(const std::string &directory, g2o::HyperGraph *graph)
  {
    std::ifstream ifs(directory + "/data");
    if (!ifs)
    {
      return false;
    }

    long node_id = -1;
    boost::optional<Eigen::Isometry3d> estimate;

    while (!ifs.eof())
    {
      std::string token;
      ifs >> token;

      if (token == "stamp")
      {
        ifs >> stamp.sec >> stamp.nsec;
      }
      else if (token == "estimate")
      {
        Eigen::Matrix4d mat;
        for (int i = 0; i < 4; i++)
        {
          for (int j = 0; j < 4; j++)
          {
            ifs >> mat(i, j);
          }
        }
        estimate = Eigen::Isometry3d::Identity();
        estimate->linear() = mat.block<3, 3>(0, 0);
        estimate->translation() = mat.block<3, 1>(0, 3);
      }
      else if (token == "odom")
      {
        Eigen::Matrix4d odom_mat = Eigen::Matrix4d::Identity();
        for (int i = 0; i < 4; i++)
        {
          for (int j = 0; j < 4; j++)
          {
            ifs >> odom_mat(i, j);
          }
        }

        odom.setIdentity();
        odom.linear() = odom_mat.block<3, 3>(0, 0);
        odom.translation() = odom_mat.block<3, 1>(0, 3);
      }
      else if (token == "accum_distance")
      {
        ifs >> accum_distance;
      }
      else if (token == "id")
      {
        ifs >> node_id;
      }
    }

    if (node_id < 0)
    {
      ROS_ERROR_STREAM("invalid node id!!");
      ROS_ERROR_STREAM(directory);
      return false;
    }

    if (graph->vertices().find(node_id) == graph->vertices().end())
    {
      ROS_ERROR_STREAM("vertex ID=" << node_id << " does not exist!!");
      return false;
    }

    node = dynamic_cast<g2o::VertexSE3 *>(graph->vertices()[node_id]);
    if (node == nullptr)
    {
      ROS_ERROR_STREAM("failed to downcast!!");
      return false;
    }

    if (estimate)
    {
      node->setEstimate(*estimate);
    }

    pcl::PointCloud<PointT>::Ptr cloud_(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(directory + "/cloud.pcd", *cloud_);
    cloud_t = cloud_;

    return true;
  }

  long KeyFrame::id() const
  {
    return node->id();
  }

  Eigen::Isometry3d KeyFrame::estimate() const
  {
    return node->estimate();
  }

  KeyFrameSnapshot::KeyFrameSnapshot(const Eigen::Isometry3d &pose, const pcl::PointCloud<PointC>::ConstPtr &cloud_c) : pose(pose), cloud(cloud_c) {} // 들어온 값을 사용해서 초기화?

  KeyFrameSnapshot::KeyFrameSnapshot(const KeyFrame::Ptr &key) : pose(key->node->estimate()), cloud(key->cloud_c),leaves(key->leaves) {} // 최적화된 위치를 받아서 Pose를 초기화

  KeyFrameSnapshot::~KeyFrameSnapshot() {}

} // namespace hdl_graph_slam
