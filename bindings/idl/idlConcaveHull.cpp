/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Bradley J Chambers
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include "idl_export.h"

#include <iostream>

#include "ogrsf_frmts.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>

#include <liblas/liblas.hpp>

#include <geopcl/io/LAStoPCD.hpp>
// #include <geopcl/io/PCDtoLAS.hpp>

#ifdef __cplusplus
  extern "C" {
#endif

int
idlConcaveHullnatural(IDL_STRING *input, IDL_STRING *output, const double *alpha)
{
  // Setup point clouds

  std::cout << "Computing concave hull of " << IDL_STRING_STR(input) << " with an alpha of " << *alpha << " and writing result as " << IDL_STRING_STR(output) << std::endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZI>);

  liblas::Header header;
  geopcl::LAStoPCD(IDL_STRING_STR(input), header, *cloud);

  // effectively demean the data
  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    cloud->points[i].x -= header.GetOffsetX();
    cloud->points[i].y -= header.GetOffsetY();
    cloud->points[i].z -= header.GetOffsetZ();
  }

  // Setup OGR
  const char *driver_name = "KML";
  OGRSFDriver *driver;
  
  OGRRegisterAll();

  driver = OGRSFDriverRegistrar::GetRegistrar()->GetDriverByName(driver_name);
  if (driver == NULL)
  {
    std::cout << driver_name << " driver not available." << std::endl;
    return 1;
  }

  OGRDataSource *source;

  source = driver->CreateDataSource("boundary.kml", NULL);
  if (source == NULL)
  {
    std::cout << "Creation of output file failed." << std::endl;
    return 1;
  }

  OGRSpatialReference input_srs;
  input_srs.SetFromUserInput(header.GetSRS().GetWKT().c_str());

  std::cout << header.GetSRS().GetWKT() << std::endl;

  OGRSpatialReference *output_srs;
  output_srs = input_srs.CloneGeogCS();

  OGRCoordinateTransformation *UTMtoLL = OGRCreateCoordinateTransformation(&input_srs, output_srs);
  if (UTMtoLL == NULL)
  {
    std::cout << "Error creating UTM to Lat/Lon transformation." << std::endl;
    return 1;
  }

  OGRLayer *layer;

  layer = source->CreateLayer( "polygon_out", output_srs, wkbPolygon, NULL);
  if (layer == NULL)
  {
    std::cout << "Layer creation failed." << std::endl;
    return 1;
  }

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

  // Create the concave hull
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::ConcaveHull<pcl::PointXYZI> chull;
  chull.setInputCloud(cloud_projected);
  chull.setAlpha(*alpha);
  chull.reconstruct(*cloud_hull);

  std::cerr << "Concave hull has: " << cloud_hull->points.size()
            << " data points." << std::endl;

//  geopcl::PCDtoLAS(IDL_STRING_STR(output), header, *cloud_hull);

  OGRFeature *feature;

  feature = OGRFeature::CreateFeature(layer->GetLayerDefn());

  OGRLinearRing ring;
  
  for (size_t i = 0; i < cloud_hull->points.size(); ++i)
  {
    double lat, lon;
    lat = cloud_hull->points[i].y + header.GetOffsetY();
    lon = cloud_hull->points[i].x + header.GetOffsetX();

//    std::cout << lon << " " << lat << std::endl;

    if (UTMtoLL->Transform(1, &lon, &lat))
    {
      OGRPoint pt;

      pt.setX(lon);
      pt.setY(lat);

//      std::cout << 

      ring.addPoint(&pt);
    }
  }

  OGRPolygon poly;
  
  poly.addRing(&ring);
      
  if (feature->SetGeometry(&poly) != OGRERR_NONE)
  {
    std::cout << "Failed to set geometry in feature." << std::endl;
    return 1;
  }

  if (layer->CreateFeature(feature) != OGRERR_NONE)
  {
    std::cout << "Failed to create feature in shapefile." << std::endl;
    return 1;
  }

  OGRFeature::DestroyFeature(feature);

  OGRDataSource::DestroyDataSource(source);

  return 0;
}

int
idlConcaveHull(int argc, void *argv[])
{
  if (argc != 3)
  {
    std::cerr << "Required arguments: input.las output.las <alpha>" << std::endl;
    return 1;
  }

  return idlConcaveHullnatural((IDL_STRING *) argv[0], (IDL_STRING *) argv[1], static_cast<double *>(argv[2]));
}

#ifdef __cplusplus
  }
#endif

