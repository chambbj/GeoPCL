#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <geopcl/io/PDALtoPCD.hpp>

int main(int argc, char** argv)
{
  if (argc < 3)
  {
    std::cerr << "Required arguments: input.las output.pcd" << std::endl;
    return 1;
  }

  std::string input = argv[1];
  std::string output = argv[2];

  std::cout << "Reading " << input << " and writing " << output << std::endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

  geopcl::PDALtoPCD(input, *cloud);

  pcl::io::savePCDFileASCII(output.c_str(), *cloud);

  return 0;
}

