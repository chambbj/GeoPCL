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

#include <stdio.h>
#include "idl_export.h"

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <liblas/liblas.hpp>

#include <geopcl/io/LAStoPCD.hpp>
#include <geopcl/io/PCDtoLAS.hpp>

#ifdef __cplusplus
extern "C" {
#endif

  int
  idlVoxelGridnatural(IDL_STRING *input, IDL_STRING *output)
  {
    // Setup point clouds
    std::cout << "Decimating " << IDL_STRING_STR(input) << " and writing result as " << IDL_STRING_STR(output) << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

    liblas::Header header;
    geopcl::LAStoPCD(IDL_STRING_STR(input), header, *cloud);

    std::cout << "Point cloud before filtering: " << cloud->points.size()
              << " data points." << std::endl;

    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*cloud_filtered);

    std::cout << "Point cloud after filtering: " << cloud_filtered->points.size()
              << " data points." << std::endl;

std::cout << "version minor: " << header.GetVersionMinor() << std::endl;
std::cout << "format: " << header.GetDataFormatId() << std::endl;
std::cout << "x scale / offset: " << header.GetScaleX() << " " << header.GetOffsetX() << std::endl;
std::cout << "y scale / offset: " << header.GetScaleY() << " " << header.GetOffsetY() << std::endl;
std::cout << "z scale / offset: " << header.GetScaleZ() << " " << header.GetOffsetZ() << std::endl;

    geopcl::PCDtoLAS(IDL_STRING_STR(output), header, *cloud_filtered);

    return 0;
  }

  int
  idlVoxelGrid(int argc, void *argv[])
  {
    if (argc != 2)
    {
      std::cerr << "Required arguments: input.las output.las" << std::endl;
      return 1;
    }

    return idlVoxelGridnatural((IDL_STRING *) argv[0], (IDL_STRING *) argv[1]);
  }

#ifdef __cplusplus
}
#endif

