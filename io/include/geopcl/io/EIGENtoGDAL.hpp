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

#ifndef INCLUDED_EIGENtoGDAL_HPP
#define INCLUDED_EIGENtoGDAL_HPP

#include <string>

#include <Eigen/Core>

#include "gdal_priv.h"
#include "ogr_spatialref.h"

namespace geopcl
{
  /**
   * \brief Convert Eigen matrix to GDAL raster.
   *
   * Converts Eigen matrix to GDAL raster.
   *
   * future add wkt input option, format, datatype, etc.
   */
  template <typename MatrixT>
  void EIGENtoGDAL(MatrixT &m, const std::string &output)
  {
    // Get rows and columns from the input matrix
    int rows = m.rows();
    int cols = m.cols();
    
    // Setup GDAL
    GDALAllRegister();

    const char* pszFormat = "GTiff";
    char **papszMetadata;
    char **papszOptions = NULL;
    char *pszDstFilename = const_cast<char *>(output.c_str());
    GDALDriver *poDriver;
    GDALDataset *poDstDS;
    GDALRasterBand *poBand;
    
    // Attempt to get the driver
    poDriver = GetGDALDriverManager()->GetDriverByName(pszFormat);
    if (poDriver == NULL)
    {
      std::cerr << "Could not get driver " << pszFormat << "." << std::endl;
      return;
    }
    
    // Check that the driver supports the Create() method
    papszMetadata = poDriver->GetMetadata();
    if (!CSLFetchBoolean(papszMetadata, GDAL_DCAP_CREATE, FALSE))
    {
      std::cerr << "Driver " << pszFormat << " does not support Create() method." << std::endl;
      return;
    }
    
    // Create the raster output
    poDstDS = poDriver->Create(pszDstFilename, cols, rows, 1, GDT_Float32, papszOptions);
    
    // Write data to the raster
    poBand = poDstDS->GetRasterBand(1);
    poBand->RasterIO( GF_Write, 0, 0, cols, rows, m.data(), cols, rows, GDT_Float32, 0, 0);
    
    // Close the raster
    GDALClose((GDALDatasetH) poDstDS);
  }
}  // geopcl

#endif  // INCLUDED_EIGENtoGDAL_HPP

