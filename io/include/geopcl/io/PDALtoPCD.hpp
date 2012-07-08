/*
 * Inspired, and partially borrowed from VTK_PCL_Conversions
 * https://github.com/daviddoria/VTK_PCL_Conversions
 */

#ifndef INCLUDED_PDALtoPCD_HPP
#define INCLUDED_PDALtoPCD_HPP

#include <fstream>
#include <string>

#include <boost/filesystem.hpp>

#include <pdal/Dimension.hpp>
#include <pdal/FileUtils.hpp>
#include <pdal/Options.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/Reader.hpp>
#include <pdal/Schema.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/StageIterator.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_traits.h>

namespace geopcl
{
  /**
   * \brief Convert PDAL point cloud to PCD.
   *
   * Converts PDAL data to PCD format.
   */
  template <typename CloudT>
  void PDALtoPCD(const std::string &input, CloudT &cloud)
  {
    pdal::Options options;
    {
      options.add<std::string>("filename", input);
    }

//    const std::string inputFile = options.getValueOrThrow<std::string>("filename");

    if (!pdal::FileUtils::fileExists(input))
    {
      std::cerr << "file not found: " << input << std::endl;
      return;
    }

    std::string ext = boost::filesystem::extension(input);

    if (ext == "")
    {
      std::cerr << "could not detect driver from extension" << std::endl;
      return;
    }

    ext = ext.substr(1, ext.length() - 1);

    if (ext == "")
    {
      std::cerr << "could not detect driver from extension" << std::endl;
      return;
    }

    boost::to_lower(ext);

//    pdal::Option &fn = options.getOptionByRef("filename");
//    fn.setValue<std::string>(filename);

    std::map<std::string, std::string> drivers;
    drivers["las"] = "drivers.las.reader";
    drivers["laz"] = "drivers.las.reader";
    drivers["bin"] = "drivers.terrasolid.reader";
    drivers["qi"] = "drivers.qfit.reader";
    drivers["xml"] = "drivers.pipeline.reader";
    drivers["nitf"] = "drivers.nitf.reader";
    drivers["ntf"] = "drivers.nitf.reader";
//    drivers["bpf"] = "drivers.bpf.reader";

    std::string driver = drivers[ext];

    if (driver == "")
    {
      std::cerr << "Cannot determine file type of " << input << std::endl;
      return;
    }

    pdal::StageFactory factory;
    pdal::Stage *reader = factory.createReader(driver, options);

    if (!reader)
    {
      std::cerr << "reader creation failed" << std::endl;
      return;
    }

    try
    {
      reader->initialize();
    }
    catch (std::exception const &e)
    {
      std::cerr << "Cannot open " << input << " for read. Exiting..." << std::endl;
      return;
    }

    typedef typename pcl::traits::fieldList<typename CloudT::PointType>::type FieldList;

    const pdal::Schema &schema = reader->getSchema();

    cloud.width = reader->getNumPoints();
    cloud.height = 1;  // unorganized point cloud
    cloud.is_dense = false;
    cloud.points.resize(cloud.width);

    typename CloudT::PointType testPoint = cloud.points[0];

    bool has_x = false; bool has_y = false; bool has_z = false;
    float x_val = 0.0f; float y_val = 0.0f; float z_val = 0.0f;
    pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint, "x", has_x, x_val));
    pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint, "y", has_y, y_val));
    pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint, "z", has_z, z_val));

    pdal::PointBuffer data(schema, reader->getNumPoints());
    pdal::StageSequentialIterator *iter = reader->createSequentialIterator(data);
    boost::uint32_t numRead = iter->read(data);

    const pdal::Schema &buffer_schema = data.getSchema();
    const pdal::Dimension &dX = buffer_schema.getDimension("X");
    const pdal::Dimension &dY = buffer_schema.getDimension("Y");
    const pdal::Dimension &dZ = buffer_schema.getDimension("Z");

    if (has_x && has_y && has_z)
    {
      for (size_t i = 0; i < cloud.points.size(); ++i)
      {
        typename CloudT::PointType p = cloud.points[i];
        pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "x", data.getField<float>(dX, i)));
        pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "y", data.getField<float>(dY, i)));
        pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "z", data.getField<float>(dZ, i)));
        cloud.points[i] = p;
      }
    }

    boost::optional<pdal::Dimension const &> dI = buffer_schema.getDimensionOptional("Intensity");

    bool has_i = false;
    float i_val = 0.0f;
    pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint, "intensity", has_i, i_val));

    if (has_i && dI)
    {
      for (size_t i = 0; i < cloud.points.size(); ++i)
      {
        typename CloudT::PointType p = cloud.points[i];
        pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "intensity", data.getField<float>(*dI, i)));
        cloud.points[i] = p;
      }
    }
  }
}  // geopcl

#endif  // INCLUDED_PDALtoPCD_HPP

