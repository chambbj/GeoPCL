/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Bradley J Chambers
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef INCLUDED_POINTS_TO_GRID_HPP
#define INCLUDED_POINTS_TO_GRID_HPP

#include <iostream>
#include <limits>

#include <Eigen/Core>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace geopcl
{
  /**
   * \brief Rasterize point cloud.
   *
   * Rasterize point cloud.
   */
  template <typename CloudT, typename Derived>
  void PointsToGrid(const CloudT &cloud, float resolution, const Eigen::MatrixBase<Derived> &grid_)
  {
    std::cout << resolution << " meters" << std::endl;

    /*
     * Compute extents of points.
     */
    typename CloudT::PointType min_pt, max_pt; //testPoint = cloud.points[0];
    pcl::getMinMax3D(cloud, min_pt, max_pt);
    std::cout << min_pt << std::endl;
    std::cout << max_pt << std::endl;

    /*
     * Compute size of grid.
     */
    boost::int32_t cols = std::ceil((max_pt.x - min_pt.x) / resolution);
    boost::int32_t rows = std::ceil((max_pt.y - min_pt.y) / resolution);
    std::cout << cols << " x bins (cols)" << std::endl;
    std::cout << rows << " y bins (rows)" << std::endl;

    /*
     * Initialize grid.
     */
    Eigen::MatrixBase<Derived> &grid = const_cast<Eigen::MatrixBase<Derived>& >(grid_);
    grid.derived().resize(rows, cols);
    grid.setConstant(std::numeric_limits<boost::int32_t>::max());

    /*
     * Loop over point cloud and compute grid minimums.
     */
    for (size_t i = 0; i < cloud.points.size(); ++i)
    {
      typename CloudT::PointType p = cloud.points[i];

      boost::int32_t xbin = std::floor((p.x - min_pt.x) / resolution);
      boost::int32_t ybin = std::floor((p.y - min_pt.y) / resolution);

      if (p.z < grid(ybin, xbin))
      {
        grid(ybin, xbin) = p.z;
      }
    }
  }
}  // geopcl

#endif  // INCLUDED_POINTS_TO_GRID_HPP

