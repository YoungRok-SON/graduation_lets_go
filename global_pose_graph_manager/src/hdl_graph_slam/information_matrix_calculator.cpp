// SPDX-License-Identifier: BSD-2-Clause

#include <hdl_graph_slam/information_matrix_calculator.hpp>

#include <pcl/search/kdtree.h>
#include <pcl/common/transforms.h>

namespace hdl_graph_slam {

// Constructor
InformationMatrixCalculator::InformationMatrixCalculator(ros::NodeHandle& nh) 
{
  use_const_inf_matrix = nh.param<bool>("use_const_inf_matrix", false);
  const_stddev_x = nh.param<double>("const_stddev_x", 0.5);
  const_stddev_q = nh.param<double>("const_stddev_q", 0.1);

  var_gain_a = nh.param<double>("var_gain_a", 20.0);
  min_stddev_x = nh.param<double>("min_stddev_x", 0.1);
  max_stddev_x = nh.param<double>("max_stddev_x", 5.0);
  min_stddev_q = nh.param<double>("min_stddev_q", 0.05);
  max_stddev_q = nh.param<double>("max_stddev_q", 0.2);
  fitness_score_thresh = nh.param<double>("fitness_score_thresh", 0.5);
}

InformationMatrixCalculator::~InformationMatrixCalculator() {}

Eigen::MatrixXd InformationMatrixCalculator::calc_information_matrix(const pcl::PointCloud<PointT>::ConstPtr& cloud1, const pcl::PointCloud<PointT>::ConstPtr& cloud2, const Eigen::Isometry3d& relpose) const 
{
  if(use_const_inf_matrix) 
  {
    Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
    inf.topLeftCorner(3, 3).array() /= const_stddev_x;
    inf.bottomRightCorner(3, 3).array() /= const_stddev_q;
    return inf;
  }
  // 두 포인트 클라우드 cloud1 및 cloud2 간의 적합도 점수를 계산합니다. 이 점수는 두 포인트 클라우드가 얼마나 잘 매칭되는지를 나타내는 지표
  double fitness_score = calc_fitness_score(cloud1, cloud2, relpose);

  // 특정 상황에 따른 분산의 최솟값 및 최댓값을 계산
  double min_var_x = std::pow(min_stddev_x, 2);
  double max_var_x = std::pow(max_stddev_x, 2);
  double min_var_q = std::pow(min_stddev_q, 2);
  double max_var_q = std::pow(max_stddev_q, 2);

  // 각각 평행 이동 위치와 회전에 대한 가중치를 계산하는 데 사용
  float w_x = weight(var_gain_a, fitness_score_thresh, min_var_x, max_var_x, fitness_score);
  float w_q = weight(var_gain_a, fitness_score_thresh, min_var_q, max_var_q, fitness_score);

  Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6); // 6x6 항등 행렬을 생성
  inf.topLeftCorner(3, 3).array() /= w_x;                // 정보 행렬의 좌측 상단 3x3 행렬은 위치 (x, y, z)에 관련된 정보
  inf.bottomRightCorner(3, 3).array() /= w_q;            // 정보 행렬의 우측 하단 3x3 행렬은 회전에 관련된 정보
  return inf;
  // 정보 행렬은 주어진 두 포인트 클라우드 간의 상대적 위치 및 회전에 관한 불확실성을 나타냄
  // 정보 행렬의 값이 크면 해당 차원의 불확실성이 낮다는 것을 의미
  // 값이 작으면 불확실성이 높다는 것을 의미
}

double InformationMatrixCalculator::calc_fitness_score(const pcl::PointCloud<PointT>::ConstPtr& cloud1, const pcl::PointCloud<PointT>::ConstPtr& cloud2, const Eigen::Isometry3d& relpose, double max_range) 
{
  pcl::search::KdTree<PointT>::Ptr tree_(new pcl::search::KdTree<PointT>());
  tree_->setInputCloud(cloud1);

  double fitness_score = 0.0;

  // Transform the input dataset using the final transformation
  pcl::PointCloud<PointT> input_transformed;
  pcl::transformPointCloud(*cloud2, input_transformed, relpose.cast<float>());

  std::vector<int> nn_indices(1);
  std::vector<float> nn_dists(1);

  // For each point in the source dataset
  int nr = 0;
  for(size_t i = 0; i < input_transformed.points.size(); ++i) 
  {
    // Find its nearest neighbor in the target
    tree_->nearestKSearch(input_transformed.points[i], 1, nn_indices, nn_dists);

    // Deal with occlusions (incomplete targets)
    if(nn_dists[0] <= max_range) 
    {
      // Add to the fitness score
      fitness_score += nn_dists[0];  // 이 fitness_score라는 값이 작을수록 좋음
      // 아니 fitness_score면 잘 맞을수록 높아야하는거 아닌가? ㅋㅋㅋ
      nr++;
    }
  }

  if(nr > 0)
    return (fitness_score / nr); // 평균값을 리턴
  else
    return (std::numeric_limits<double>::max()); // Max_range보다 가까운 점이 없다면 최대값 반환
}

}  // namespace hdl_graph_slam
