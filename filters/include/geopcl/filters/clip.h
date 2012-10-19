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

#ifndef INCLUDED_CLIP_H
#define INCLUDED_CLIP_H

#include <pcl/filters/filter.h>  // Filter, input_, indices_
#include <pcl/point_cloud.h>     // PointCloud

namespace geopcl
{
  /**
   * \brief Clip is derived from the pcl::Filter class and is used to
   * clip point cloud data.
   */
  template<typename PointT>
  class ClipFilter : public pcl::Filter<PointT>
  {
    using pcl::Filter<PointT>::input_;  // for setInputCloud

    typedef typename pcl::PointCloud<PointT> PointCloud;

    float m_xmin;
    float m_xmax;
    float m_ymin;
    float m_ymax;
    float m_zmin;
    float m_zmax;

  public:
    // CREATORS
    ClipFilter()
      : m_xmin(-std::numeric_limits<float>::max())
      , m_xmax(std::numeric_limits<float>::max())
      , m_ymin(-std::numeric_limits<float>::max())
      , m_ymax(std::numeric_limits<float>::max())
      , m_zmin(-std::numeric_limits<float>::max())
      , m_zmax(std::numeric_limits<float>::max())
    {
    }

    // MANIPULATORS
    inline void
    setXMin(const float &xmin)
    { m_xmin = xmin; }

    inline void
    setXMax(const float &xmax)
    { m_xmax = xmax; }

    inline void
    setYMin(const float &ymin)
    { m_ymin = ymin; }

    inline void
    setYMax(const float &ymax)
    { m_ymax = ymax; }

    inline void
    setZMin(const float &zmin)
    { m_zmin = zmin; }

    inline void
    setZMax(const float &zmax)
    { m_zmax = zmax; }

    // ACCESSORS
    inline float
    getXMin()
    { return m_xmin; }

    inline float
    getXMax()
    { return m_xmax; }

    inline float
    getYMin()
    { return m_ymin; }

    inline float
    getYMax()
    { return m_ymax; }

    inline float
    getZMin()
    { return m_zmin; }

    inline float
    getZMax()
    { return m_zmax; }

  private:

  protected:
    /**
     * \brief Filter the input data and store the results into output
     *
     * \param[out] output the resultant point cloud
     */
    void
    applyFilter(PointCloud &output);
  };
} // geopcl

#endif  // INCLUDED_CLIP_H

