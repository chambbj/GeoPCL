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

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

#include <pcl/point_types.h>

#include <geopcl/filters/clip.h>
#include <geopcl/io/LAStoPCD.hpp>
#include <geopcl/io/PCDtoLAS.hpp>

int
main(int argc, char **argv)
{

  if (argc < 9)
  {
    std::cerr << "Required arguments: input.las output.las xmin xmax ymin ymax zmin zmax" << std::endl;
    return 1;
  }

  fs::path input = argv[1];
  fs::path output = argv[2];
  float xmin = atof(argv[3]);
  float xmax = atof(argv[4]);
  float ymin = atof(argv[5]);
  float ymax = atof(argv[6]);
  float zmin = atof(argv[7]);
  float zmax = atof(argv[8]);

  if (fs::is_regular_file(input) && (fs::extension(input) == ".las"))
  {
    std::cout << "Reading " << input << " and writing " << output << std::endl;

    /*
     * Read the point cloud XYZ coordinates only.
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr icloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ocloud(new pcl::PointCloud<pcl::PointXYZI>);

    liblas::Header header;

    geopcl::LAStoPCD(input.string().c_str(), header, *icloud);

    /*
     * Create the clipping filter, subtracting offsets from the user
     * specified clipping bounds.
     */
    geopcl::ClipFilter<pcl::PointXYZI> clip;
    clip.setXMin(xmin - header.GetOffsetX());
    clip.setXMax(xmax - header.GetOffsetX());
    clip.setYMin(ymin - header.GetOffsetY());
    clip.setYMax(ymax - header.GetOffsetY());
    clip.setZMin(zmin - header.GetOffsetZ());
    clip.setZMax(zmax - header.GetOffsetZ());

    /*
     * Specify the input point cloud and filter.
     */
    clip.setInputCloud(icloud);
    clip.filter(*ocloud);

    if (ocloud->points.size() == 0)
    {
      std::cerr << "Unable to create clipped point cloud." << std::endl;
      return (1);
    }

    /*
     * Write the filtered cloud to disk.
     */
    geopcl::PCDtoLAS(output.string().c_str(), header, *ocloud);
  }

  return (0);
}

