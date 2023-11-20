// SPDX-License-Identifier: BSD-2-Clause

#ifndef ORB_SLAM2_REGISTRATIONS_HPP
#define ORB_SLAM2_REGISTRATIONS_HPP

#include <ros/ros.h>

#include <pcl/registration/registration.h>

namespace ORB_SLAM2 {

pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr resgistrationNDTOMP();

}  // namespace hdl_graph_slam

#endif  //ORB_SLAM2_REGISTRATIONS_HPP
