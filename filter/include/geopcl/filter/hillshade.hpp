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

#ifndef INCLUDED_HILLSHADE_HPP
#define INCLUDED_HILLSHADE_HPP

#include <cmath>

#include <boost/cstdint.hpp>

#include <Eigen/Core>

namespace geopcl
{
  static const double kPI = 3.14159265358979323846264338327950288;

  /**
   * \brief Create hillshade.
   *
   * http://webhelp.esri.com/arcgisdesktop/9.2/index.cfm?TopicName=How%20Hillshade%20works
   */
  template <typename Derived>
  void Hillshade(const Eigen::MatrixBase<Derived> &surface, const double &altitude, const double &azimuth, const Eigen::MatrixBase<Derived> &hillshade_)
  {
    double zenith_deg = 90.0 - altitude;
    double zenith_rad = zenith_deg * kPI / 180.0;

    double azimuth_math = 360.0 - azimuth + 90.0;

    if (azimuth_math > 360.0) azimuth_math -= 360.0;

    double azimuth_rad = azimuth_math * kPI / 180.0;

    double z_factor = 1.0;

    double cellsize = 5.0;

    Eigen::MatrixBase<Derived> &hillshade = const_cast<Eigen::MatrixBase<Derived>& >(hillshade_);
    hillshade.derived().resize(surface.rows(), surface.cols());
    hillshade.setConstant(0);

    for (boost::int32_t r = 1; r < surface.rows() - 1; ++r)
    {
      for (boost::int32_t c = 1; c < surface.cols() - 1; ++c)
      {
        double dzdx = ((surface(r - 1, c + 1) + 2 * surface(r, c + 1) + surface(r + 1, c + 1)) -
                       (surface(r - 1, c - 1) + 2 * surface(r, c - 1) + surface(r + 1, c - 1))) /
                      (8 * cellsize);

        double dzdy = ((surface(r + 1, c - 1) + 2 * surface(r + 1, c) + surface(r + 1, c + 1)) -
                       (surface(r - 1, c - 1) + 2 * surface(r - 1, c) + surface(r - 1, c + 1))) /
                      (8 * cellsize);

        double slope_rad = std::atan(z_factor * std::sqrt(dzdx * dzdx + dzdy * dzdy));

        double aspect_rad = 0.0;

        if (std::abs(dzdx) > 0.0)
        {
          aspect_rad = std::atan2(dzdy, -dzdx);

          if (aspect_rad < 0.0)
          {
            aspect_rad = 2 * kPI + aspect_rad;
          }
        }
        else
        {
          if (dzdy > 0.0)
          {
            aspect_rad = kPI / 2.0;
          }
          else if (dzdy < 0.0)
          {
            aspect_rad = 2.0 * kPI - kPI / 2.0;
          }

          // else aspect_rad = aspect_rad; ??
        }

        hillshade(r, c) = 255.0 * ((std::cos(zenith_rad) * std::cos(slope_rad)) +
                                   (std::sin(zenith_rad) * std::sin(slope_rad) * std::cos(azimuth_rad - aspect_rad)));
      }
    }
  }
}  // geopcl

#endif  // INCLUDED_HILLSHADE_HPP

