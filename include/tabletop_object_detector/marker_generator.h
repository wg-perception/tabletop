/*********************************************************************
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/

// Author(s): Marius Muja and Matei Ciocarlie

#ifndef _MARKER_GENERATOR_H_
#define _MARKER_GENERATOR_H_

#include <vector>

#include <opencv2/core/core.hpp>

#include <visualization_msgs/Marker.h>
#include <shape_msgs/Mesh.h>
#include <geometry_msgs/Pose.h>

namespace tabletop_object_detector {

//! A convenience class for generating markers based on various clustering and fitting data
/*! Just a place to group all the different debug marker generators
  so they don't polute other classes.
*/
class MarkerGenerator {
 public:
  //! Create a line strip marker that goes around a detected table
  static visualization_msgs::Marker getTableMarker(float xmin, float xmax, float ymin, float ymax);
  //! A marker with all the points in a cloud in a random color
  static visualization_msgs::Marker getCloudMarker(const std::vector<cv::Vec3f>& cloud);
  //! A marker showing where a fit model is believed to be
  static visualization_msgs::Marker getFitMarker(const shape_msgs::Mesh &mesh, double rank);
  //! A marker showing where a convex hull table is
  static visualization_msgs::Marker getConvexHullTableMarker(const shape_msgs::Mesh &mesh);
  //! Create a generic Marker
  static visualization_msgs::Marker createMarker(std::string frame_id, double duration, double xdim, double ydim, double zdim,
					  double r, double g, double b, int type, int id, std::string ns, geometry_msgs::Pose pose);
};

}//namespace

#endif
