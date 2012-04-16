#ifndef INCLUDED_LAStoPCD_H
#define INCLUDED_LAStoPCD_H

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
   * Converts LAS data to a coordinate-only PointXYZ PCD format.
   */
  template <typename CloudT>
  void LAStoPCD(const std::string &input, CloudT &cloud)
  {
    std::ifstream ifs;
    if(!liblas::Open(ifs, input.c_str()))
    {
      std::cerr << "Cannot open " << input << " for read. Exiting..." << std::endl;
      return;
    }

    liblas::ReaderFactory f;
    liblas::Reader reader = f.CreateWithStream(ifs);

    typedef typename pcl::traits::fieldList<typename CloudT::PointType>::type FieldList;

    cloud.width = reader.GetHeader().GetPointRecordsCount();
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
        liblas::Point const& q = reader.GetPoint();

        typename CloudT::PointType p = cloud.points[i];
        pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "x", q.GetX()));
        pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "y", q.GetY()));
        pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "z", q.GetZ()));
        cloud.points[i] = p;
      }
    }

    // same for intensity

    // same for RGB
  }
}  // geopcl

#endif  // INCLUDED_LAStoPCD_H

