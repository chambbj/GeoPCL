#include <iostream>

#include <liblas/liblas.hpp>

#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <geopcl/io/PCDtoLAS.hpp>
#include <geopcl/io/LAStoPCD.hpp>

int main(int argc, char **argv)
{
  if (argc < 3)
  {
    std::cerr << "Required arguments: input.las output.las" << std::endl;
    return 1;
  }

  std::string input = argv[1];
  std::string output = argv[2];

  std::cout << "Reading " << input << " and writing " << output << std::endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

  liblas::Header header;

  geopcl::LAStoPCD(input, header, *cloud);

  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 1.0);
  pass.filter(*cloud);

  geopcl::PCDtoLAS(output, header, *cloud);

  return 0;
}

