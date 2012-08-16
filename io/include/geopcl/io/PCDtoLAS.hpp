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

#ifndef INCLUDED_PCDtoLAS_HPP
#define INCLUDED_PCDtoLAS_HPP

#include <fstream>
#include <string>

#include <boost/foreach.hpp>

#include <liblas/liblas.hpp>
#include <liblas/utility.hpp>
#include <liblas/external/property_tree/ptree.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_traits.h>

namespace geopcl
{
  /**
   * \brief Convert PCD point cloud to LAS.
   *
   * Converts PCD data to LAS format.
   */
  template <typename CloudT>
  void PCDtoLAS(const std::string &output, liblas::Header &header, const CloudT &cloud)
  {
    typename CloudT::PointType testPoint = cloud.points[0];

    typedef typename pcl::traits::fieldList<typename CloudT::PointType>::type FieldList;

    std::ofstream ofs;

    if (!liblas::Create(ofs, output.c_str()))
    {
      std::cerr << "Cannot open " << output << " for write. Exiting..." << std::endl;
      return;
    }

//    liblas::PointFormatName format = liblas::ePointFormat0;
    // not quite ready to handle the other point formats

//    header.SetVersionMinor(2);
//    header.SetDataFormatId(format);

//    double scale = 1.0;
//    header.SetScale(scale, scale, scale);
//    header.SetOffset(0.0, 0.0, 0.0);
    header.SetPointRecordsCount(cloud.points.size());

    liblas::Writer *writer = new liblas::Writer(ofs, header);

    liblas::CoordinateSummary *summary(new liblas::CoordinateSummary);

    bool has_i = false;
    float i_val = 0.0f;
    pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint, "intensity", has_i, i_val));

    for (size_t i = 0; i < cloud.points.size(); ++i)
    {
      liblas::Point q;

      typename CloudT::PointType p = cloud.points[i];
      q.SetX(static_cast<double>(p.x));
      q.SetY(static_cast<double>(p.y));
      q.SetZ(static_cast<double>(p.z));

      if (has_i)
      {
        q.SetIntensity(static_cast<boost::uint16_t>(p.intensity));
      }

      try
      {
        summary->AddPoint(q);
        writer->WritePoint(q);
      }
      catch (std::exception)
      {
        std::cerr << "Point writing failed!" << std::endl;
      }
    }

    // Repair header
    for (boost::uint32_t i = 0; i < 5; i++)
    {
      header.SetPointRecordsByReturnCount(i, 0);
    }

    liblas::property_tree::ptree tree = summary->GetPTree();

    try
    {
      header.SetMin(tree.get<double>("summary.points.minimum.x"),
                    tree.get<double>("summary.points.minimum.y"),
                    tree.get<double>("summary.points.minimum.z"));
      header.SetMax(tree.get<double>("summary.points.maximum.x"),
                    tree.get<double>("summary.points.maximum.y"),
                    tree.get<double>("summary.points.maximum.z"));
    }
    catch (liblas::property_tree::ptree_bad_path const &)
    {
      std::cerr << "Unable to write bounds info." << std::endl;
      return;
    }

    try
    {
      for (boost::uint32_t i = 0; i < 5; i++)
      {
        header.SetPointRecordsByReturnCount(i, 0);
      }

      BOOST_FOREACH(ptree::value_type & v,
                    tree.get_child("summary.points.points_by_return"))
      {
        boost::uint32_t i = v.second.get<boost::uint32_t>("id");
        boost::uint32_t count = v.second.get<boost::uint32_t>("count");
        header.SetPointRecordsByReturnCount(i - 1, count);
      }
    }
    catch (liblas::property_tree::ptree_bad_path const &)
    {
      std::cerr << "Unable to write header point count info." << std::endl;
      return;
    }

    if (writer != 0)
    {
      delete writer;
    }

    ofs.close();

    // same for RGB - although there is a mismatch between PCL's assumption
    // that RGB will be 8 bits/channel and LAS's specification that RGB will be
    // 16 bits/channel
  }
}  // geopcl

#endif  // INCLUDED_PCDtoLAS_HPP

