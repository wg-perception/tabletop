/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#include <fstream>
#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include <ecto/ecto.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tabletop/table/tabletop_segmenter.h>

using ecto::tendrils;

namespace tabletop
{
  /** Ecto implementation of a module that takes
   *
   */
  struct ConvexHull
  {
    static void
    declare_params(ecto::tendrils& params)
    {
      params.declare(&ConvexHull::cluster_tolerance_, "cluster_tolerance",
                     "The distance used as a threshold when finding a plane", 0.2);
      params.declare(&ConvexHull::vertical_direction_, "vertical_direction", "The vertical direction");
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&ConvexHull::cloud_in_, "cloud", "The point cloud in which to find a table.");

      inputs.declare(&ConvexHull::cloud_hull_, "cloud", "The convex hull of the biggest cluster of the points.");
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
    }

    /** Get the 2d keypoints and figure out their 3D position from the depth map
     * @param inputs
     * @param outputs
     * @return
     */
    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      TabletopHull tabletop_hull(*cluster_tolerance_, *vertical_direction_);

      *cloud_hull_ = tabletop_hull.cluster<pcl::PointXYZ>(*cloud_in_);

      return ecto::OK;
    }
  private:
    ecto::spore<float> cluster_tolerance_;

    /** flag indicating whether we run in debug mode */
    ecto::spore<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> cloud_in_;
    /** flag indicating whether we run in debug mode */
    ecto::spore<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> cloud_hull_;
    /** The vertical direction */
    ecto::spore<Eigen::Vector3f> vertical_direction_;
  };
}

ECTO_CELL(tabletop_table, tabletop::ConvexHull, "ConvexHull", "Given a table and inliers, find the convex hull.");
