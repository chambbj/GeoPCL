/*
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
  void PCDtoLAS(const std::string &output, CloudT &cloud)
  {
    typename CloudT::PointType testPoint = cloud.points[0];

    typedef typename pcl::traits::fieldList<typename CloudT::PointType>::type FieldList;

    std::ofstream ofs;
    if(!liblas::Create(ofs, output.c_str()))
    {
      std::cerr << "Cannot open " << output << " for write. Exiting..." << std::endl;
      return;
    }

    // need to handle the header a bit more intelligently, e.g., assuming the
    // following workflow:
    //
    // LAStoPCD(fname, *cloud);
    // ...some work on *cloud
    // PCDtoLAS(fname, *cloud);
    //
    // we should probably return the header in the first step, and pass it to
    // the final step to retain as much file level information as possible
    liblas::Header header;

    liblas::PointFormatName format = liblas::ePointFormat0;
    // not quite ready to hanlde the other point formats

    header.SetVersionMinor(2);
    header.SetDataFormatId(format);

    double scale = 1.0;
    header.SetScale(scale, scale, scale);
    header.SetOffset(0.0, 0.0, 0.0);
    header.SetPointRecordsCount(cloud.points.size());

    liblas::Writer* writer = new liblas::Writer(ofs, header);

    liblas::CoordinateSummary* summary(new liblas::CoordinateSummary);

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
    catch (liblas::property_tree::ptree_bad_path const&)
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

      BOOST_FOREACH(ptree::value_type &v,
        tree.get_child("summary.points.points_by_return"))
      {
        boost::uint32_t i = v.second.get<boost::uint32_t>("id");
        boost::uint32_t count = v.second.get<boost::uint32_t>("count");
        header.SetPointRecordsByReturnCount(i-1, count);
      }
    }
    catch (liblas::property_tree::ptree_bad_path const&)
    {
      std::cerr << "Unable to write header point count info." << std::endl;
      return;
    }

    if (writer != 0)
    {
      delete writer;
    }

//    if (ofs != 0)
//    {
//      liblas::Cleanup(ofs);
//    }

//    liblas::Header hnew = liblas::FetchHeader(output.c_str());
//    liblas::RepairHeader(*summary, hnew);
//    liblas::RewriteHeader(hnew, output.c_str());

    // same for RGB - although there is a mismatch between PCL's assumption
    // that RGB will be 8 bits/channel and LAS's specification that RGB will be
    // 16 bits/channel
  }
}  // geopcl

#endif  // INCLUDED_PCDtoLAS_HPP

