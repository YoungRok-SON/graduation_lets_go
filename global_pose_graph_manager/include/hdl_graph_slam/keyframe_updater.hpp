// SPDX-License-Identifier: BSD-2-Clause

#ifndef KEYFRAME_UPDATER_HPP
#define KEYFRAME_UPDATER_HPP

#include <ros/ros.h>
#include <Eigen/Dense>


namespace hdl_graph_slam {

/**
 * @brief this class decides if a new frame should be registered to the pose graph as a keyframe
 */
class KeyframeUpdater {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief constructor
   * @param pnh
   */
  KeyframeUpdater(ros::NodeHandle& pnh) 
  {
    keyframe_delta_trans         = pnh.param<double>("global_pose_graph_manager/keyframe_delta_trans", 2.0);
    keyframe_delta_angle         = pnh.param<double>("global_pose_graph_manager/keyframe_delta_angle", 2.0);
    vehicle_num_                 = pnh.param<int>("global_pose_graph_manager/vehicle_num", 2);
    accumulated_delta_anchor_    = pnh.param<double>("global_pose_graph_manager/keyframe_delta_anchor", 5.0);
    vec_accumulated_distance_.resize(vehicle_num_);
    vec_prev_keypose_.resize(vehicle_num_);
    is_first_.resize(vehicle_num_);
    vec_accumulated_distance_anchor_.resize(vehicle_num_);
    vec_is_new_anchor_.resize(vehicle_num_);
    vec_prev_anchor_pose_.resize(vehicle_num_);
    for (size_t vehicle_idx = 0; vehicle_idx < vehicle_num_; vehicle_idx++)
    {
      vec_accumulated_distance_[vehicle_idx] = 0.0;
      vec_prev_keypose_[vehicle_idx] = Eigen::Isometry3d::Identity();
      is_first_[vehicle_idx] = true;
      vec_accumulated_distance_anchor_[vehicle_idx] = 0.0;
      vec_is_new_anchor_[vehicle_idx] = false;
      vec_prev_anchor_pose_[vehicle_idx] = Eigen::Isometry3d::Identity();
    }
    
  }

  /**
   * @brief decide if a new frame should be registered to the graph
   * @param pose  pose of the frame
   * @return  if true, the frame should be registered
   */
  bool update(const Eigen::Isometry3d& pose, int vehicle_id) 
  {
    for (size_t vehicle_idx = 0; vehicle_idx < vehicle_num_; vehicle_idx++)
    {
      if (vehicle_idx != vehicle_id)
        continue;

    
      // first frame is always registered to the graph
      if(is_first_[vehicle_idx]) 
      {
        is_first_[vehicle_idx] = false;
        vec_prev_keypose_[vehicle_idx] = pose;
        vec_prev_anchor_pose_[vehicle_idx] = pose;
        return true;
      }

      // calculate the delta transformation from the previous keyframe
      Eigen::Isometry3d delta = vec_prev_keypose_[vehicle_idx].inverse() * pose;
      Eigen::Isometry3d delta_anchor = vec_prev_anchor_pose_[vehicle_idx].inverse() * pose;
      double dx = delta.translation().norm();
      double dx_anchor = delta_anchor.translation().norm();
      double da = Eigen::AngleAxisd(delta.linear()).angle();

      // too close to the previous frame
      if(dx < keyframe_delta_trans && da < keyframe_delta_angle) {
        return false;
      }

      vec_accumulated_distance_[vehicle_idx] += dx; // Keyframe으로써 추가가 되는 것이 확정되면 누적 거리를 증가
      vec_accumulated_distance_anchor_[vehicle_idx] += dx_anchor;
      vec_prev_keypose_[vehicle_idx] = pose;
      vec_prev_anchor_pose_[vehicle_idx] = pose;

      // Anchor Set by distance threshold
      if ( vec_accumulated_distance_anchor_[vehicle_idx] > accumulated_delta_anchor_)
      {
        vec_is_new_anchor_[vehicle_idx] = true;
        vec_accumulated_distance_anchor_[vehicle_idx] = 0.0;
      }
      return true;
    }
  }

  /**
   * @brief the last keyframe's accumulated distance from the first keyframe
   * @return accumulated distance
   */
  double get_accum_distance(int vehicle_num) const 
  {
    return vec_accumulated_distance_[vehicle_num];
  }

  bool is_new_anchor(int vehicle_num)
  {
    if( vec_is_new_anchor_[vehicle_num] == true)
    {
      vec_is_new_anchor_[vehicle_num] = false;
      return true;
    }
    else
      return false;
  }

private:
  // parameters
  double keyframe_delta_trans;  //
  double keyframe_delta_angle;  //
  int vehicle_num_;
  double accumulated_delta_anchor_;

  std::vector<bool> is_first_;
  std::vector<double> vec_accumulated_distance_;
  std::vector<Eigen::Isometry3d> vec_prev_keypose_;
  std::vector<Eigen::Isometry3d> vec_prev_anchor_pose_;
  std::vector<double> vec_accumulated_distance_anchor_;
  std::vector<bool> vec_is_new_anchor_;
};

}  // namespace hdl_graph_slam

#endif  // KEYFRAME_UPDATOR_HPP
