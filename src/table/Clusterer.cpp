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
  struct Clusterer
  {
    static void
    declare_params(ecto::tendrils& params)
    {
      float c_limits[6] =
      { std::numeric_limits<float>::min(), std::numeric_limits<float>::max(), std::numeric_limits<float>::min(),
        std::numeric_limits<float>::max(), std::numeric_limits<float>::min(), std::numeric_limits<float>::max() };
      std::vector<float> limits(c_limits, c_limits + 6);
      params.declare(&Clusterer::filter_limits_, "filter_limits",
                     "The limits of the interest box to find a table, in order [xmin,xmax,ymin,ymax,zmin,zmax]",
                     limits);
      params.declare(&Clusterer::min_cluster_size_, "min_cluster_size",
                     "The minimum number of points deemed necessary to find a table.", 1000);
      params.declare(&Clusterer::plane_detection_voxel_size_, "plane_detection_voxel_size",
                     "The size of a voxel cell when downsampling ", 0.01);
      params.declare(&Clusterer::normal_k_search_, "normal_k_search",
                     "The number of nearest neighbors to use when computing normals", 10);
      params.declare(&Clusterer::plane_threshold_, "plane_threshold",
                     "The distance used as a threshold when finding a plane", 0.2);
      params.declare(&Clusterer::vertical_direction_, "vertical_direction", "The vertical direction");
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&Clusterer::cloud_in_, "cloud", "The point cloud in which to find a table.");

      outputs.declare(&Clusterer::table_coefficients_, "coefficients", "The coefficients of the table.");
      inputs.declare(&Clusterer::cloud_out_, "cloud", "The point cloud in which to find a table.");
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
      TabletopSegmenter<pcl::PointXYZ> table_segmenter(*filter_limits_, *min_cluster_size_,
                                                       *plane_detection_voxel_size_, *normal_k_search_,
                                                       *plane_threshold_);
      table_segmenter.findTable(*cloud_in_, *table_coefficients_, *cloud_out_);

      return ecto::OK;
    }
  private:
    /** The limits of the interest box to find a table, in order [xmin,xmax,ymin,ymax,zmin,zmax] */
    ecto::spore<std::vector<float> > filter_limits_;
    /** The minimum number of points deemed necessary to find a table */
    ecto::spore<size_t> min_cluster_size_;
    /** The size of a voxel cell when downsampling */
    ecto::spore<float> plane_detection_voxel_size_;
    /** The number of nearest neighbors to use when computing normals */
    ecto::spore<unsigned int> normal_k_search_;
    /** The distance used as a threshold when finding a plane */
    ecto::spore<float> plane_threshold_;

    /** The input cloud */
    ecto::spore<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> cloud_in_;
    /** The output cloud */
    ecto::spore<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_out_;
    /** The minimum number of inliers in order to do pose matching */
    ecto::spore<pcl::ModelCoefficients::Ptr> table_coefficients_;
    /** The vertical direction */
    ecto::spore<Eigen::Vector3f> vertical_direction_;
  };
}

ECTO_CELL(tabletop_table, tabletop::Clusterer, "Clusterer", "Given a point cloud, find  a potential table.");
