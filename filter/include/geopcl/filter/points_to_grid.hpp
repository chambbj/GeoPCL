#ifndef INCLUDED_POINTS_TO_GRID_HPP
#define INCLUDED_POINTS_TO_GRID_HPP

//#include <fstream>
#include <iostream>
#include <limits>
//#include <string>

//#include <liblas/liblas.hpp>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl/point_traits.h>

namespace geopcl
{
  /**
   * \brief Rasterize point cloud.
   *
   * Rasterize point cloud.
   */
  template <typename CloudT>
  void PointsToGrid(const CloudT &cloud, float resolution, CloudT &grid)
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
    boost::int32_t xsize = std::ceil((max_pt.x - min_pt.x) / resolution);
    boost::int32_t ysize = std::ceil((max_pt.y - min_pt.y) / resolution);
    std::cout << xsize << " x bins" << std::endl;
    std::cout << ysize << " y bins" << std::endl;

    /*
     * Initialize grid.
     */
    grid.width = xsize;
    grid.height = ysize;
    grid.is_dense = false;
    grid.points.resize(cloud.width*grid.height);
    std::cout << grid.points.size() << " points" << std::endl;

    for (size_t x = 0; x < xsize; ++x)
    {
      for (size_t y = 0; y < ysize; ++y)
      {
        size_t idx = x + (y * xsize);
        grid.points[idx].x = min_pt.x + (x * resolution);
        grid.points[idx].y = min_pt.y + (y * resolution);
        grid.points[idx].z = std::numeric_limits<float>::max();
      }
    }
    std::cout << grid.points[0] << std::endl;

    /*
     * Loop over point cloud and compute grid minimums.
     */
    for (size_t i = 0; i < cloud.points.size(); ++i)
    {
      typename CloudT::PointType p = cloud.points[i];

      boost::int32_t xbin = std::floor((p.x - min_pt.x) / resolution);
      boost::int32_t ybin = std::floor((p.y - min_pt.y) / resolution);

      size_t idx = xbin + (ybin * xsize);

      if (p.z < grid.points[idx].z) grid.points[idx].z = p.z;
    }
    std::cout << grid.points[0] << std::endl;
  }
}  // geopcl

#endif  // INCLUDED_POINTS_TO_GRID_HPP

