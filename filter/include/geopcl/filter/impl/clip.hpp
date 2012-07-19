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

#ifndef INCLUDED_CLIP_IMPL_H
#define INCLUDED_CLIP_IMPL_H

#include "geopcl/filter/clip.h"

#include <vector>

#include <boost/cstdint.hpp>

#include <Eigen/Core>

#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>

template<typename PointT> void
geopcl::ClipFilter<PointT>::applyFilter(PointCloud &output)
{
  output = *input_;

  // Clip to the appropriate 3D extents
  Eigen::Vector4f min_pt, max_pt;
  min_pt << getXMin(), getYMin(), getZMin(), 0;
  max_pt << getXMax(), getYMax(), getZMax(), 0;

  std::vector<boost::int32_t> p_indices;
  p_indices.clear();
  pcl::getPointsInBox((*input_), min_pt, max_pt, p_indices);

  if (p_indices.size() == 0)
  {
    std::cerr << "No points found in the specified bounding box!" << std::endl;
    output.points.clear();
    return;
  }

  pcl::IndicesPtr p_indices_ptr(new std::vector<boost::int32_t> (p_indices));

  // Setup the extract filter
  pcl::ExtractIndices<PointT> extract;
  extract.setNegative(false);
  extract.setInputCloud(input_);
  extract.setIndices(p_indices_ptr);
  extract.filter(output);
}

#endif  // INCLUDED_CLIP_IMPL_H

