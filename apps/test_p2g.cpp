#include <iostream>

#include <liblas/liblas.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <geopcl/io/PCDtoLAS.hpp>
#include <geopcl/io/LAStoPCD.hpp>
//#include <geopcl/filter/points_to_grid.hpp>
#include <geopcl/filter/p2g.hpp>

int main(int argc, char** argv)
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
  pcl::PointCloud<pcl::PointXYZI>::Ptr grid(new pcl::PointCloud<pcl::PointXYZI>);

  liblas::Header header;

  geopcl::LAStoPCD(input, header, *cloud);

  geopcl::PointsToGrid(*cloud, 0.3f, *grid);

  geopcl::PCDtoLAS(output, header, *grid);

  return 0;
}

