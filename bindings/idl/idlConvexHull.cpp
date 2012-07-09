#include <stdio.h>
#include "idl_export.h"

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>

#include <liblas/liblas.hpp>

#include <geopcl/io/LAStoPCD.hpp>
#include <geopcl/io/PCDtoLAS.hpp>

int
idlConvexHullnatural(IDL_STRING *input, IDL_STRING *output)
{
  std::cout << "Computing concave hull of " << IDL_STRING_STR(input) << " and writing result as " << IDL_STRING_STR(output) << std::endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZI>);

  liblas::Header header;
  geopcl::LAStoPCD(IDL_STRING_STR(input), header, *cloud);

  // Create a set of planar coefficients with X=0, Y=0, Z=1, i.e., the XY plane
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  coefficients->values.resize(4);
  coefficients->values[0] = coefficients->values[1] = 0;
  coefficients->values[2] = 1.0;
  coefficients->values[3] = 0;

  // Create the filtering object
  pcl::ProjectInliers<pcl::PointXYZI> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(cloud);
  proj.setModelCoefficients(coefficients);
  proj.filter(*cloud_projected);

  // Create the convex hull
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::ConvexHull<pcl::PointXYZI> chull;
  chull.setInputCloud(cloud_projected);
  chull.reconstruct(*cloud_hull);

  std::cerr << "Convex hull has: " << cloud_hull->points.size()
            << " data points." << std::endl;

  geopcl::PCDtoLAS(IDL_STRING_STR(output), header, *cloud_hull);

  return 0;
}

int
idlConvexHull(int argc, void *argv[])
{
  if (argc != 2)
  {
    std::cerr << "Required arguments: input.las output.las" << std::endl;
    return 1;
  }

  return idlConvexHullnatural((IDL_STRING *) argv[0], (IDL_STRING *) argv[1]);
}

