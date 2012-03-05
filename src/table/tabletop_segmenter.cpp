/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <tabletop/table/tabletop_segmenter.h>

namespace tabletop
{
  /** Assumes plane coefficients are of the form ax+by+cz+d=0, normalized
   * @param plane_coefficients
   * @param up_direction_in
   * @param flatten_plane if true, the plane coefficients are modified so that up_direction_in is the normal
   */
  void
  getPlaneTransform(const Eigen::Vector4f &plane_coefficients, const Eigen::Vector3f &up_direction, bool flatten_plane,
                    Eigen::Vector3f & translation, Eigen::Matrix3f & rotation)
  {
    double a = plane_coefficients[0], b = plane_coefficients[1], c = plane_coefficients[2], d = plane_coefficients[3];
    // assume plane coefficients are normalized
    translation = Eigen::Vector3f(-a * d, -b * d, -c * d);
    Eigen::Vector3f z(a, b, c);

    //if we are flattening the plane, make z just be up_direction
    if (flatten_plane)
    {
      z = up_direction;
    }
    else
    {
      //make sure z points "up"
      if (z.dot(up_direction) < 0)
      {
        z = -1.0 * z;
      }
      z = -1.0 * z;
    }

    //try to align the x axis with the x axis of the original frame
    //or the y axis if z and x are too close too each other
    Eigen::Vector3f x(1, 0, 0);
    if (fabs(z.dot(x)) > 1.0 - 1.0e-4)
      x = Eigen::Vector3f(0, 1, 0);
    Eigen::Vector3f y = z.cross(x).normalized();
    x = y.cross(z).normalized();

    rotation << x.coeff(0), x.coeff(1), x.coeff(2), y.coeff(0), y.coeff(1), y.coeff(2), z.coeff(0), z.coeff(1), z.coeff(
        2);
    rotation.transposeInPlace();
  }
}
