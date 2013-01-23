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

#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>

#include <object_recognition_core/common/pose_result.h>
#include "tabletop_segmenter.h"

using object_recognition_core::common::PoseResult;

using ecto::tendrils;

namespace tabletop
{
  /** Ecto implementation of a module that takes
   *
   */
  struct TablePose
  {
    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&TablePose::table_coefficients_, "table_coefficients", "The coefficients of planar surfaces.").required(true);

      outputs.declare(&TablePose::pose_results_, "pose_results", "The results of object recognition");
    }

    /** Compute the pose of the table plane
     * @param inputs
     * @param outputs
     * @return
     */
    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      pose_results_->resize(table_coefficients_->size());
      for (size_t i = 0; i < table_coefficients_->size(); ++i)
      {
        cv::Matx33f R;
        cv::Vec3f T;
        getPlaneTransform((*table_coefficients_)[i], T, R);
        PoseResult pose_result;
        pose_result.set_R(cv::Mat(R));
        pose_result.set_T(cv::Mat(T));
        (*pose_results_)[i] = pose_result;
      }

      return ecto::OK;
    }
  private:
  /** The coefficients of the tables */
  ecto::spore<std::vector<cv::Vec4f> > table_coefficients_;

    ecto::spore<std::vector<PoseResult> > pose_results_;
  };
}

ECTO_CELL(tabletop_table, tabletop::TablePose, "TablePose", "Given a point cloud, find  a potential table.");
