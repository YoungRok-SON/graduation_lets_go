// SPDX-License-Identifier: BSD-2-Clause

#include <registrations.hpp>

#include <iostream>

#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>
#include <fast_gicp/gicp/fast_gicp.hpp>

namespace ORB_SLAM2
{

  pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr resgistrationNDTOMP()
  {
    std::string nn_search_method = "DIRECT7";
    float ndt_resolution = 0.4;
    int num_threads = 8;
    std::cout << "registration: NDT_OMP " << nn_search_method << " " << ndt_resolution << " (" << num_threads << " threads)" << std::endl;

    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt(new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>()); // NDT_OMP와 NDT와 다른점은 pclomp::NormalDistributionsTransform를 사용
    if (num_threads > 0)
    {
      ndt->setNumThreads(num_threads);
    }
    ndt->setTransformationEpsilon(0.05);
    ndt->setMaximumIterations(64);
    ndt->setResolution(ndt_resolution);
    if (nn_search_method == "KDTREE")
    {
      ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
    }
    else if (nn_search_method == "DIRECT1")
    {
      ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
    }
    else
    {
      ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    }
    
    return ndt;
  }


} // namespace ORB_SLAM2