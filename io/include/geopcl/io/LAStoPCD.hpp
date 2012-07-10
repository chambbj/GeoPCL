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
 *
 * Inspired, and partially borrowed from VTK_PCL_Conversions
 * https://github.com/daviddoria/VTK_PCL_Conversions
 */

#ifndef INCLUDED_LAStoPCD_HPP
#define INCLUDED_LAStoPCD_HPP

#include <fstream>
#include <string>

#include <liblas/liblas.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_traits.h>

namespace geopcl
{
  /**
   * \brief Convert LAS point cloud to PCD.
   *
   * Converts LAS data to PCD format.
   */
  template <typename CloudT>
  void LAStoPCD(const std::string &input, liblas::Header &header, CloudT &cloud)
  {
    std::ifstream ifs;

    if (!liblas::Open(ifs, input.c_str()))
    {
      std::cerr << "Cannot open " << input << " for read. Exiting..." << std::endl;
      return;
    }

    liblas::ReaderFactory f;
    liblas::Reader reader = f.CreateWithStream(ifs);

    typedef typename pcl::traits::fieldList<typename CloudT::PointType>::type FieldList;

    header = reader.GetHeader();

    cloud.width = header.GetPointRecordsCount();
    cloud.height = 1;  // unorganized point cloud
    cloud.is_dense = false;
    cloud.points.resize(cloud.width);

    typename CloudT::PointType testPoint = cloud.points[0];

    bool has_x = false; bool has_y = false; bool has_z = false;
    float x_val = 0.0f; float y_val = 0.0f; float z_val = 0.0f;
    pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint, "x", has_x, x_val));
    pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint, "y", has_y, y_val));
    pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint, "z", has_z, z_val));

    if (has_x && has_y && has_z)
    {
      for (size_t i = 0; i < cloud.points.size(); ++i)
      {
        reader.ReadNextPoint();
        liblas::Point const &q = reader.GetPoint();

        typename CloudT::PointType p = cloud.points[i];
        pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "x", static_cast<float>(q.GetX())));
        pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "y", static_cast<float>(q.GetY())));
        pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "z", static_cast<float>(q.GetZ())));
        cloud.points[i] = p;
      }
    }

    bool has_i = false;
    float i_val = 0.0f;
    pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint, "intensity", has_i, i_val));

    if (has_i)
    {
      reader.Reset();

      for (size_t i = 0; i < cloud.points.size(); ++i)
      {
        reader.ReadNextPoint();
        liblas::Point const &q = reader.GetPoint();

        typename CloudT::PointType p = cloud.points[i];
        pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "intensity", static_cast<float>(q.GetIntensity())));
        cloud.points[i] = p;
      }
    }

    // same for RGB - although there is a mismatch between PCL's assumption
    // that RGB will be 8 bits/channel and LAS's specification that RGB will be
    // 16 bits/channel
  }
}  // geopcl

#endif  // INCLUDED_LAStoPCD_HPP

