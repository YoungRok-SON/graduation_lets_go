// SPDX-License-Identifier: BSD-2-Clause

#ifndef LOOP_DETECTOR_HPP
#define LOOP_DETECTOR_HPP

#include <boost/format.hpp>
#include <hdl_graph_slam/keyframe.hpp>
#include <hdl_graph_slam/registrations.hpp>
#include <hdl_graph_slam/graph_slam.hpp>

#include <hdl_graph_slam/map_cloud_generator.hpp>

#include <g2o/types/slam3d/vertex_se3.h>

namespace hdl_graph_slam
{

  struct Loop
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr = std::shared_ptr<Loop>;

    Loop(const KeyFrame::Ptr &key1, const KeyFrame::Ptr &key2, const Eigen::Matrix4f &relpose) : key1(key1), key2(key2), relative_pose(relpose) {}

  public:
    KeyFrame::Ptr key1;
    KeyFrame::Ptr key2;
    Eigen::Matrix4f relative_pose;
  };

  /**
   * @brief this class finds loops by scan matching and adds them to the pose graph
   */
  class LoopDetector
  {
  public:
    typedef pcl::PointXYZI PointT;

    /**
     * @brief constructor
     * @param pnh
     */
    LoopDetector(ros::NodeHandle &pnh)
    {
      distance_thresh = pnh.param<double>("distance_thresh", 5.0);
      accum_distance_thresh = pnh.param<double>("accum_distance_thresh", 8.0); // 첫번째 Keyframe으로부터의 누적 이동 거리
      distance_from_last_edge_thresh = pnh.param<double>("min_edge_interval", 5.0); // 마지막으로 생성된 에지와의 누적거리!

      fitness_score_max_range = pnh.param<double>("fitness_score_max_range", std::numeric_limits<double>::max()); // 이건 왜 필요한거지
      fitness_score_thresh = pnh.param<double>("fitness_score_thresh", 0.5);

      /* New Feature */
      use_submap_loop_         = pnh.param<bool>("use_submap_loop", false);
      nr_submap_keyframe_      = pnh.param<int>("nr_submap_keyframe", 8);
      map_cloud_resolution_    = pnh.param<double>("map_cloud_resolution", 0.05);
      registration             = select_registration_method(pnh); // 정합 알고리즘
      last_edge_accum_distance = 0.0;
      map_cloud_generator_.reset(new MapCloudGenerator()); // New Feature

    }

    /**
     * @brief detect loops and add them to the pose graph
     * @param keyframes       keyframes
     * @param new_keyframes   newly registered keyframes
     * @param graph_slam      pose graph
     */
    std::vector<Loop::Ptr> detect( const std::vector<KeyFrame::Ptr> &keyframes, const std::deque<KeyFrame::Ptr> &new_keyframes, hdl_graph_slam::GraphSLAM &graph_slam)
    {
      std::vector<Loop::Ptr> detected_loops; // Loop 객체는 key1, key2, relpose로 구성
      for (const auto &new_keyframe : new_keyframes) // 새로운 키프레임에 있는 정보를 하나씩 가져와 비교하는 듯?
      {
        std::vector<KeyFrame::Ptr> candidates;
        if(use_submap_loop_)
          candidates = find_submap_keyframes(keyframes, new_keyframe); // Only One Candidate
        else
          candidates = find_candidates(keyframes, new_keyframe); // 후보자들을 먼저 찾고
        
        auto loop = matching(candidates, new_keyframe, graph_slam); // 후보자들을 매칭해서 스코어 기준으로 나누는듯?

        if (loop) // nullptr이 아니면
        {
          detected_loops.push_back(loop);
        }
      }

      return detected_loops;
    }

    double get_distance_thresh() const
    {
      return distance_thresh;
    }

  private:
    /**
     * @brief find closest one loop candidates. A detected loop begins at one of #keyframes and ends at #new_keyframe
     * @param keyframes      candidate keyframes of loop start
     * @param new_keyframe   loop end keyframe
     * @return Only one loop candidate
     */
    std::vector<KeyFrame::Ptr> find_submap_keyframes(const std::vector<KeyFrame::Ptr> &keyframes, const KeyFrame::Ptr &new_keyframe) // const
    {

      // too close to the last registered loop edge
      if (new_keyframe->accum_distance - last_edge_accum_distance < distance_from_last_edge_thresh) 
      { // 새로운 키프레임의 누적거리에서 마지막으로 에지가 생성되었을 때의 거리를 뺀 값이 
        // 지정한 값보다 작으면 그 키프레임은 넘기기
        return std::vector<KeyFrame::Ptr>();
      }

      std::vector<KeyFrame::Ptr> candidates;
      candidates.reserve(32); // GPT한테 물어보자

      int distance_closest = distance_thresh;
      KeyFrame::Ptr closest_keyframe;
      int keyframe_idx = 0;
      int closest_keyframe_idx = 0;
      for (const auto &k : keyframes)
      {
        keyframe_idx++;
        // traveled distance between keyframes is too small
        if (new_keyframe->accum_distance - k->accum_distance < accum_distance_thresh)
        { // 새로 들어온 keyframe의 누적거리와 기존 keyframe의 누적 거리가 너무 가까우면
          continue;
        }

        const auto &pos1 = k->node->estimate().translation();
        const auto &pos2 = new_keyframe->node->estimate().translation();

        // estimated distance between keyframes is too small
        double dist = (pos1.head<2>() - pos2.head<2>()).norm(); // x,y 데이터만 사용
        if (dist > distance_thresh) // 지정한 거리보다 멀다면
        { 
          continue;
        }

        // 루프 돌면서 가장 작은 애들만 반환
        if( dist < distance_closest )
        {
          distance_closest = dist;
          closest_keyframe = k;
          closest_keyframe_idx = keyframe_idx;
        }
      }

      // Get possible front and back number of accessible keyframes.
      int accessible_front = closest_keyframe_idx - nr_submap_keyframe_;
      int accessible_back = closest_keyframe_idx + nr_submap_keyframe_;
      target_keyframe_idx_ = nr_submap_keyframe_; // normal state
      if( accessible_front < 0 ) // not enough front keyframes
      {
        target_keyframe_idx_ = nr_submap_keyframe_ + accessible_front; 
        accessible_front = 0;
      }
      if( accessible_back > keyframes.size() ) // not enough back keyframes
      {
        target_keyframe_idx_ = accessible_back - nr_submap_keyframe_; 
        accessible_back = keyframes.size();

      }
      ROS_INFO("keyframe size: %ld", keyframes.size());
      ROS_INFO("accessible_front Idx: %d", accessible_front);
      ROS_INFO("accessible_back Idx: %d", accessible_back);
      ROS_INFO("Target Keyframe Idx: %d", target_keyframe_idx_);
      // Get a Submap accessible keyframes
      for (int i = accessible_front; i < accessible_back; i++)
        candidates.push_back(keyframes[i]);

      return candidates;
    }

    /**
     * @brief find loop candidates. A detected loop begins at one of #keyframes and ends at #new_keyframe
     * @param keyframes      candidate keyframes of loop start
     * @param new_keyframe   loop end keyframe
     * @return loop candidates
     */
    std::vector<KeyFrame::Ptr> find_candidates(const std::vector<KeyFrame::Ptr> &keyframes, const KeyFrame::Ptr &new_keyframe) const
    {
      // too close to the last registered loop edge
      if (new_keyframe->accum_distance - last_edge_accum_distance < distance_from_last_edge_thresh) 
      { // 새로운 키프레임의 누적거리에서 마지막으로 에지가 생성되었을 때의 거리를 뺀 값이 
        // 지정한 값보다 작으면 그 키프레임은 넘기기
        return std::vector<KeyFrame::Ptr>();
      }

      std::vector<KeyFrame::Ptr> candidates;
      candidates.reserve(32); // GPT한테 물어보자

      for (const auto &k : keyframes)
      {
        // traveled distance between keyframes is too small
        if (new_keyframe->accum_distance - k->accum_distance < accum_distance_thresh)
        { // 새로 들어온 keyframe의 누적거리와 기존 keyframe의 누적 거리가 너무 가까우면
          continue;
        }

        const auto &pos1 = k->node->estimate().translation();
        const auto &pos2 = new_keyframe->node->estimate().translation();

        // estimated distance between keyframes is too small
        double dist = (pos1.head<2>() - pos2.head<2>()).norm(); // x,y 데이터만 사용
        if (dist > distance_thresh) // 지정한 거리보다 멀다면
        { 
          continue;
        }

        // 결국 지정한 거리보다 멀지만, 실제적인 거리가 가까운 애들만 후보자 벡터네 넣어 반환
        candidates.push_back(k);
      }

      return candidates;
    }


    /**
     * @brief To validate a loop candidate this function applies a scan matching between keyframes consisting the loop. If they are matched well, the loop is added to the pose graph
     * @param candidate_keyframes  candidate keyframes of loop start
     * @param new_keyframe         loop end keyframe
     * @param graph_slam           graph slam
     */
    Loop::Ptr matching(const std::vector<KeyFrame::Ptr> &candidate_keyframes, const KeyFrame::Ptr &new_keyframe, hdl_graph_slam::GraphSLAM &graph_slam)
    {
      if (candidate_keyframes.empty())
      { // 후보 키프레임이 없다면
        return nullptr;
      }

      registration->setInputTarget(new_keyframe->cloud); // 새롭게 들어온 키프레임을 기준으로

      double best_score = std::numeric_limits<double>::max(); // 최저값을 넣는게 아니라 최대값?
      KeyFrame::Ptr best_matched; // 제일 스코어가 높은 키프레임 선정
      Eigen::Matrix4f relative_pose; // 두 키프레임 사이의 상대 변환

      std::cout << std::endl;
      ROS_INFO( "--- loop detection ---" "<< std::endl");
      ROS_INFO( "num_candidates: %ld", candidate_keyframes.size());
      std::cout << "matching" << std::flush; // flush 사용하면 즉시 출력 가능. 아래 연산 때문에 출력이 늦어지는 일이 없게..
      auto t1 = ros::Time::now();

      pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
      pcl::PointCloud<PointT>::Ptr submap_cloud(new pcl::PointCloud<PointT>());

      // New Feature Testing
      if (use_submap_loop_)
      {
        ROS_INFO("Map Cloud Generation");
        // Generate Submap from keyframes
        submap_cloud = map_cloud_generator_->generate(candidate_keyframes, map_cloud_resolution_); // 0.5는 resolution
        // Try Matching with Submap
          registration->setInputSource(submap_cloud); // 매칭시킬 포인트 클라우드를 가져옴
          Eigen::Isometry3d new_keyframe_estimate = new_keyframe->node->estimate(); // 타겟의 값 위치 값 가져옴
          new_keyframe_estimate.linear() = Eigen::Quaterniond(new_keyframe_estimate.linear()).normalized().toRotationMatrix();
          // Get Closest Keyframe Estimate
          // Eigen::Isometry3d candidate_estimate = candidate_keyframes[target_keyframe_idx_]->node->estimate();
          // candidate_estimate.linear() = Eigen::Quaterniond(candidate_estimate.linear()).normalized().toRotationMatrix();
          // Eigen::Matrix4f guess = (new_keyframe_estimate.inverse() * candidate_estimate).matrix().cast<float>();
          Eigen::Matrix4f guess = new_keyframe_estimate.matrix().cast<float>();
          guess(2, 3) = 0.0; // 높이를 없애는 역할.. 이게 있어도 되나..?
          // 각 노드의 위치를 통해서 guess를 구함. 즉, 두 노드 사이의 상대 위치를 구함


          registration->align(*aligned, guess); // 결국 정렬된 포인트 클라우드는 사용을 안함
          std::cout << "." << std::flush;

          best_score = registration->getFitnessScore(fitness_score_max_range);
          best_matched = KeyFrame::Ptr(new KeyFrame(candidate_keyframes[target_keyframe_idx_]->stamp, candidate_keyframes[target_keyframe_idx_]->odom, candidate_keyframes[target_keyframe_idx_]->accum_distance, submap_cloud));
          relative_pose = registration->getFinalTransformation();
      }
      else
      {
        for (const auto &candidate : candidate_keyframes)
        {
          registration->setInputSource(candidate->cloud); // 매칭시킬 포인트 클라우드를 가져옴
          Eigen::Isometry3d new_keyframe_estimate = new_keyframe->node->estimate(); // 타겟의 값 위치 값 가져옴
          new_keyframe_estimate.linear() = Eigen::Quaterniond(new_keyframe_estimate.linear()).normalized().toRotationMatrix();
          Eigen::Isometry3d candidate_estimate = candidate->node->estimate();
          candidate_estimate.linear() = Eigen::Quaterniond(candidate_estimate.linear()).normalized().toRotationMatrix();
          Eigen::Matrix4f guess = (new_keyframe_estimate.inverse() * candidate_estimate).matrix().cast<float>();
          // 각 노드의 위치를 통해서 guess를 구함. 즉, 두 노드 사이의 상대 위치를 구함

          guess(2, 3) = 0.0; // 높이를 없애는 역할.. 이게 있어도 되나..?
          registration->align(*aligned, guess); // 결국 정렬된 포인트 클라우드는 사용을 안함
          std::cout << "." << std::flush;

          double score = registration->getFitnessScore(fitness_score_max_range);
          if (!registration->hasConverged() || score > best_score) // 매칭이 제대로 안된 경우 -> 매칭이 제일 잘 된 놈만 골라내는거
          {
            continue;
          }

          best_score = score;  // Score는 낮을수록 좋은거
          best_matched = candidate; // Target이 새로 들어온 ketframe, Source가 기존에 누적한 keyframe
          relative_pose = registration->getFinalTransformation();
        }
      }

      auto t2 = ros::Time::now();
      std::cout << " done" << std::endl;
      std::cout <<  "time: " << boost::format("%.3f") % (t2 - t1).toSec() << "[sec]" << std::endl;
      ROS_INFO("best_score: %f", best_score);
      if (best_score > fitness_score_thresh)
      {
        std::cout << "loop not found..." << std::endl;
        return nullptr;
      }

      std::cout << "loop found!!" << std::endl;
      std::cout << "relpose: " << relative_pose.block<3, 1>(0, 3) << " - " << Eigen::Quaternionf(relative_pose.block<3, 3>(0, 0)).coeffs().transpose() << std::endl;

      last_edge_accum_distance = new_keyframe->accum_distance;

      return std::make_shared<Loop>(new_keyframe, best_matched, relative_pose); //key1이 source(new), key2가 target(optimized keyframes)
    }

  private:
    // New Feature
    double use_submap_loop_;
    double map_cloud_resolution_;
    int    nr_submap_keyframe_;
    std::unique_ptr<MapCloudGenerator> map_cloud_generator_;
    int    target_keyframe_idx_;


    double distance_thresh;                // estimated distance between keyframes consisting a loop must be less than this distance
    double accum_distance_thresh;          // traveled distance between ...
    double distance_from_last_edge_thresh; // a new loop edge must far from the last one at least this distance

    double fitness_score_max_range; // maximum allowable distance between corresponding points
    double fitness_score_thresh;    // threshold for scan matching

    double last_edge_accum_distance;

    pcl::Registration<PointT, PointT>::Ptr registration;
  };

} // namespace hdl_graph_slam

#endif // LOOP_DETECTOR_HPP
