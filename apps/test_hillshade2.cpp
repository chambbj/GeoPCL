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

#include <iostream>
#include <string>

#include <boost/cstdint.hpp>

#include <Eigen/Core>

//#include "gdal_priv.h"
//#include "cpl_conv.h"
//#include "cpl_string.h"
//#include "ogrsf_frmts.h"

#include <liblas/liblas.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <geopcl/filter/hillshade.hpp>
#include <geopcl/filter/points_to_grid.hpp>
#include <geopcl/io/LAStoPCD.hpp>
#include <geopcl/io/EIGENtoGDAL.hpp>
//#include <geopcl/io/GDALRaster.hpp>

int main(int argc, char **argv)
{
  if (argc < 3)
  {
    std::cerr << "Required arguments: input.las output.tiff" << std::endl;
    return 1;
  }

  std::string input = argv[1];

  std::cout << "Reading " << input << " and generating a 10 meter hillshade" << std::endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> grid;

  liblas::Header header;

  geopcl::LAStoPCD(input, header, *cloud);

  geopcl::PointsToGrid(*cloud, 10.0f, grid);

  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> hillshade;

  geopcl::Hillshade(grid, 45.0, 315.0, hillshade);

//  boost::int32_t rows = hillshade.rows();
//  boost::int32_t cols = hillshade.cols();

  std::string output = argv[2];
//  std::string format("GTiff");

  geopcl::EIGENtoGDAL(hillshade, output);


  return 0;
}

