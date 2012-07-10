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

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <liblas/liblas.hpp>

#include <geopcl/io/LAStoPCD.hpp>

#ifdef __cplusplus
  extern "C" {
#endif

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

#ifdef __cplusplus
  }
#endif

