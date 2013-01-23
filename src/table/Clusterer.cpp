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

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include <ecto/ecto.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/rgbd/rgbd.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tabletop/table/tabletop_segmenter.h>

using ecto::tendrils;

//typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;
typedef std::vector<pcl::PointCloud<PointT>, Eigen::aligned_allocator<pcl::PointCloud<PointT> > > CloudVectorType;

float pointDistanceSq(const cv::Vec3f& vec1, const cv::Vec3f& vec2)
{
  return cv::norm(vec1 - vec2) * cv::norm(vec1 - vec2);
}

  /** Cell that finds the clusters touching planes detected in a given depth image
   */
  struct Clusterer
  {
    static void
    declare_params(ecto::tendrils& params)
    {
      params.declare(&Clusterer::cluster_distance_, "cluster_distance", "The maximum distance between a point and the cluster it belongs to.",
                     0.01);
      params.declare(&Clusterer::min_cluster_size_, "min_cluster_size", "Min number of points for a cluster", 300);
      params.declare(&Clusterer::table_z_filter_min_, "table_z_filter_min",
                     "Min distance (in meters) from the table to get clusters from.", 0.01);
      params.declare(&Clusterer::table_z_filter_max_, "table_z_filter_max",
                     "Max distance (in meters) from the table to get clusters from.", 0.5);
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&Clusterer::points3d_, "points3d", "The 3dpoints as a cv::Mat_<cv::Vec3f>.");
      inputs.declare(&Clusterer::mask_, "table_mask", "The mask of the different planes.");
      inputs.declare(&Clusterer::table_coefficients_, "table_coefficients", "The coefficients of planar surfaces.");

      outputs.declare(&Clusterer::clusters_, "clusters", "For each table, a vector of clusters.");
    }

    /** Get the 2d keypoints and figure out their 3D position from the depth map
     * @param inputs
     * @param outputs
     * @return
     */
    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
    clusters_->clear();
    clusters_->resize(table_coefficients_->size());

    const cv::Mat_<cv::Vec3f> &points3d = *points3d_;

    // If an object touches a plane, its pixels also touch some pixels of the plane
    // Let's find those pixels first
    cv::Mat_<uchar> mask_binary = (*mask_) != 255, object_seeds;
    cv::dilate(mask_binary, object_seeds, cv::Mat());
    object_seeds = object_seeds - mask_binary;

    clusters_->clear();
    clusters_->resize(table_coefficients_->size());

    // For each potential pixel ...
    cv::Mat_<uchar> checked = mask_binary.clone();
    for (int y = 1; y < mask_->rows - 1; ++y) {
      uchar* iter = object_seeds.ptr<uchar>(y);
      for (int x = 1; x < mask_->cols - 1; ++x, ++iter) {
        // Only look at pixels that are on the edge of planes
        if ((!(*iter)) || (checked(y, x)))
          continue;
        // ok we have a seed, first get the plane from it
        int plane_closest = -1;
        float plane_distance_min = std::numeric_limits<float>::max();
        for (int yy = y - 1; yy <= y + 1; ++yy)
          for (int xx = x - 1; xx <= x + 1; ++xx) {
            if ((*mask_).at<uchar>(yy, xx) != 255) {
              // compute the distance from the point to the plane
              float plane_distance = pointDistanceSq(points3d(y, x), points3d(yy, xx));
              if (plane_distance < plane_distance_min) {
                plane_closest = (*mask_).at<uchar>(yy, xx);
                plane_distance_min = plane_distance;
              }
            }
          }
        if (plane_closest < 0)
          continue;
        // Create a new cluster
        pcl::PointCloud<PointT>::Ptr cluster3d(new pcl::PointCloud<PointT>);

        // Now, proceed by region growing to find the rest of the object
        std::list<cv::Point> cluster2d(1, cv::Point(x, y));
        while (!cluster2d.empty()) {
          // Look at the neighboring points
          const cv::Point& point = cluster2d.front();
          for (int yy = point.y - 1; yy <= point.y + 1; ++yy)
            for (int xx = point.x - 1; xx <= point.x + 1; ++xx) {
              if (checked(yy, xx))
                continue;
              // Compute the distance from that point to the original point
              const cv::Vec3f& point3 = points3d(yy, xx);
              if (pointDistanceSq(points3d(point.y, point.x), points3d(yy, xx)) < 0.2 * 0.2) {
                checked(yy, xx) = 1;
                cluster2d.push_back(cv::Point(xx, yy));
                cluster3d->push_back(PointT(point3[0], point3[1], point3[2]));
              }
            }
          cluster2d.pop_front();
        }

        if (cluster3d->size() < 100)
          continue;
        (*clusters_)[plane_closest].push_back(cluster3d);
      }
    }

    return ecto::OK;
  }
  private:
    /** Min distance between two clusters */
    ecto::spore<float> cluster_distance_;
    /** Min number of points for a cluster */
    ecto::spore<int> min_cluster_size_;
    /** Limits used when clustering points off the plane */
    ecto::spore<float> table_z_filter_min_;
    ecto::spore<float> table_z_filter_max_;

    /** The input cloud */
    ecto::spore<cv::Mat> points3d_;
    /** The minimum number of inliers in order to do pose matching */
    ecto::spore<std::vector<Eigen::Vector4f> > table_coefficients_;
    /** The mask of the different planes */
    ecto::spore<cv::Mat> mask_;
    /** The resulting clusters: for each table, return a vector of clusters */
    ecto::spore<std::vector<std::vector<pcl::PointCloud<PointT>::Ptr> > > clusters_;
  };

ECTO_CELL(tabletop_table, Clusterer, "Clusterer",
          "Given a point cloud and the hull of the table, find clusters.");
