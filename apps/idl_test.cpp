#include <stdio.h>
#include "idl_export.h"

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <liblas/liblas.hpp>

#include <geopcl/io/LAStoPCD.hpp>

int
idl_LAStoPCD_natural(IDL_STRING *input, IDL_STRING *output)
{ 
  std::cout << "Reading " << input << " and writing " << output << std::endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

  liblas::Header header;
  geopcl::LAStoPCD(input, header, *cloud);

  pcl::io::savePCDFileASCII(output.c_str(), *cloud);

  return 0;
}

int
idl_LAStoPCD(int argc, void *argv[])
{
  if (argc != 3)
  {
    std::cerr << "Required arguments: input.las output.pcd" << std::endl;
    return 1;
  }

  return idl_LAStoPCD_natural((IDL_STRING *) argv[0], (IDL_STRING *) argv[1]);
}

