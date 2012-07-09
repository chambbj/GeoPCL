#include <stdio.h>
#include "idl_export.h"

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <liblas/liblas.hpp>

#include <geopcl/io/LAStoPCD.hpp>

int
idlLAStoPCDnatural(IDL_STRING *input, IDL_STRING *output)
{
  std::cout << "Reading " << IDL_STRING_STR(input) << " and writing " << IDL_STRING_STR(output) << std::endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

  liblas::Header header;
  geopcl::LAStoPCD(IDL_STRING_STR(input), header, *cloud);

  pcl::io::savePCDFileASCII(IDL_STRING_STR(output), *cloud);

  return 0;
}

int
idlLAStoPCD(int argc, void *argv[])
{
  if (argc != 2)
  {
    std::cerr << "Required arguments: input.las output.pcd" << std::endl;
    return 1;
  }

  return idlLAStoPCDnatural((IDL_STRING *) argv[0], (IDL_STRING *) argv[1]);
}

