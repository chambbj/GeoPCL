#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <geopcl/io/PCDtoLAS.hpp>

int main(int argc, char** argv)
{
  if (argc < 3)
  {
    std::cerr << "Required arguments: input.pcd output.las" << std::endl;
    return 1;
  }

  std::string input = argv[1];
  std::string output = argv[2];

  std::cout << "Reading " << input << " and writing " << output << std::endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  std::cout << pcl::getFieldsList<pcl::PointXYZI>(*cloud);

  if (pcl::io::loadPCDFile<pcl::PointXYZI> (input.c_str(), *cloud) == -1)
  {
    std::cerr << "Couldn't read file" << std::endl;
    return 1;
  }

  geopcl::PCDtoLAS(output, *cloud);

  return 0;
}

