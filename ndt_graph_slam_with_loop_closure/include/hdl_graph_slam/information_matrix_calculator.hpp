// SPDX-License-Identifier: BSD-2-Clause

#ifndef INFORMATION_MATRIX_CALCULATOR_HPP
#define INFORMATION_MATRIX_CALCULATOR_HPP

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace hdl_graph_slam {

// 정보 행렬은 측정의 불확실성 (또는 정확도)을 표현하는 데 사용
class InformationMatrixCalculator 
{
public:
  using PointT = pcl::PointXYZI;

  InformationMatrixCalculator() {}
  InformationMatrixCalculator(ros::NodeHandle& nh);
  ~InformationMatrixCalculator();

  template<typename ParamServer>
  void load(ParamServer& params) 
  {
    use_const_inf_matrix = params.template param<bool>("use_const_inf_matrix", false);
    const_stddev_x = params.template param<double>("const_stddev_x", 0.5);
    const_stddev_q = params.template param<double>("const_stddev_q", 0.1);

    var_gain_a = params.template param<double>("var_gain_a", 20.0);
    min_stddev_x = params.template param<double>("min_stddev_x", 0.1);
    max_stddev_x = params.template param<double>("max_stddev_x", 5.0);
    min_stddev_q = params.template param<double>("min_stddev_q", 0.05);
    max_stddev_q = params.template param<double>("max_stddev_q", 0.2);
    fitness_score_thresh = params.template param<double>("fitness_score_thresh", 2.5);
  }

  /*
  * @brief  두 포인트 클라우드간에 가장 가까운 점을 탐색하여 거리의 평균을 구함
  * @param  cloud1 포인트 클라우드(트리에 들어감)
  * @param  cloud2 포인트 클라우드(한점씩 쿼리)
  * @param  relpose 두 포인트 클라우드 간의 상대 포즈
  * @param  max_range 매칭된 점 쌍 사이의 최대 거리값
  * @return 매칭된 점 쌍간의 평균 거리
  */
  static double calc_fitness_score(const pcl::PointCloud<PointT>::ConstPtr& cloud1, const pcl::PointCloud<PointT>::ConstPtr& cloud2, const Eigen::Isometry3d& relpose, double max_range = std::numeric_limits<double>::max());

  /* 
  * @brief 계산한 fitness_score를 기반으로 sigmoid 함수로 정규화하여 information 행렬을 생성
  * @param cloud1 포인트 클라우드1
  * @param cloud1 포인트 클라우드2 ㅎ..
  * @param relpose 두 포인트 클라우드 간의 상대 포즈
  * @return information matrix!
  */
  Eigen::MatrixXd calc_information_matrix(const pcl::PointCloud<PointT>::ConstPtr& cloud1, const pcl::PointCloud<PointT>::ConstPtr& cloud2, const Eigen::Isometry3d& relpose) const;

private:
  /* 
  * @brief       x, max_x값에 따라 주어진 범위(min_y~max_y) 내에서 sigmoid 형태의 가중치를 계산하고 반환하는 함수
  * @param a     함수의 경사도 또는 곡률 제어
  * @param max_x x의 최대값으로, 함수의 최대 범위를 정의
  * @param min_y 출력 y의 최소값으로, 결과 범위의 하한을 설정
  * @param max_y 출력 y의 최대값으로, 결과 범위의 하한을 설정
  * @param x     현재의 입력으로, 이 함수의 대상이 됨
  * @return      x값에 따른 sigmoid 가중치
  */
  double weight(double a, double max_x, double min_y, double max_y, double x) const 
  {
    double y = (1.0 - std::exp(-a * x)) / (1.0 - std::exp(-a * max_x));
    return min_y + (max_y - min_y) * y;
  }

private:
  bool use_const_inf_matrix;
  double const_stddev_x;
  double const_stddev_q;

  double var_gain_a;
  double min_stddev_x;
  double max_stddev_x;
  double min_stddev_q;
  double max_stddev_q;
  double fitness_score_thresh;
};

}  // namespace hdl_graph_slam

#endif  // INFORMATION_MATRIX_CALCULATOR_HPP
