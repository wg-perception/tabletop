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

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include <ecto/ecto.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <tabletop/Table.h>
#include <tabletop/table/tabletop_segmenter.h>

#include <object_recognition/common/pose_result.h>

using object_recognition::common::PoseResult;

using ecto::tendrils;

namespace tabletop
{
  /** Ecto implementation of a module that takes
   *
   */
  struct TablePose
  {
    static void
    declare_params(ecto::tendrils& params)
    {
      params.declare(&TablePose::up_direction_, "vertical_direction", "The vertical direction");
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&TablePose::cloud_, "cloud_hull", "The point cloud defining the table (the hull usually).");
      inputs.declare(&TablePose::table_coefficients_, "coefficients", "The coefficients of the table.");

      outputs.declare(&TablePose::pose_results_, "pose_results", "The results of object recognition");
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
      double a = (*table_coefficients_)[0], b = (*table_coefficients_)[1], c = (*table_coefficients_)[2], d =
          (*table_coefficients_)[3];
      // assume plane coefficients are normalized
      Eigen::Vector3f position(-a * d, -b * d, -c * d);
      Eigen::Vector3f z(a, b, c);

      //make sure z points "up"
      if (z.dot(*up_direction_) < 0)
      {
        z = -1.0 * z;
      }

      //try to align the x axis with the x axis of the original frame
      //or the y axis if z and x are too close too each other
      Eigen::Vector3f x(1, 0, 0);
      if (fabs(z.dot(x)) > 1.0 - 1.0e-4)
        x = Eigen::Vector3f(0, 1, 0);
      Eigen::Vector3f y = z.cross(x).normalized();
      x = y.cross(z).normalized();

      Eigen::Matrix3f rotation;
      rotation << x.coeff(0), x.coeff(1), x.coeff(2), y.coeff(0), y.coeff(1), y.coeff(2),
                               z.coeff(0), z.coeff(1), z.coeff(2);
      rotation.transposeInPlace();
      Eigen::Quaternion<float> orientation(rotation);

      PoseResult pose_result;
      pose_result.set_R(rotation);
      pose_result.set_T(position);
      pose_results_->push_back(pose_result);

      return ecto::OK;
    }
  private:
    /** flag indicating whether we run in debug mode */
    ecto::spore<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_, table_projected_ptr_, table_hull_ptr_;
    /** The minimum number of inliers in order to do pose matching */
    ecto::spore<std::vector<float> > table_coefficients_;
    /** The vertical direction */
    ecto::spore<Eigen::Vector3f> up_direction_;

    ecto::spore<std::vector<PoseResult> > pose_results_;
  };
}

ECTO_CELL(tabletop_table, tabletop::TablePose, "TablePose", "Given a point cloud, find  a potential table.");
