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

#include <ecto/ecto.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/rgbd/rgbd.hpp>

using ecto::tendrils;

namespace tabletop
{
  /** Ecto implementation of a module that takes
   *
   */
  struct TableDetector
  {
    static void
    declare_params(ecto::tendrils& params)
    {
      params.declare(&TableDetector::min_table_size_, "min_table_size",
                     "The minimum number of points deemed necessary to find a table.", 10000);
      params.declare(&TableDetector::plane_threshold_, "plane_threshold",
                     "The distance used as a threshold when finding a plane", 0.02);
      params.declare(&TableDetector::table_cluster_tolerance_, "table_cluster_tolerance",
                     "The distance used when clustering a plane", 0.2);
      params.declare(&TableDetector::up_frame_id_, "vertical_frame_id", "The vertical frame id", "/map");
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&TableDetector::points3d_, "points3d", "The 3dpoints as a cv::Mat_<cv::Vec3f>");
      inputs.declare(&TableDetector::K_, "K", "The calibration matrix");

      outputs.declare(&TableDetector::table_coefficients_, "table_coefficients", "The coefficients of planar surfaces.");
      outputs.declare(&TableDetector::table_mask_, "table_mask", "The mask of planar surfaces.");
      outputs.declare(&TableDetector::clouds_hull_, "clouds_hull", "Hulls of the samples.");
    }

  /** Get the 2d keypoints and figure out their 3D position from the depth map
   * @param inputs
   * @param outputs
   * @return
   */
  int
  process(const tendrils& inputs, const tendrils& outputs) {
    if ((points3d_->rows != prev_image_rows_)
        || (points3d_->cols != prev_image_cols_)) {
      prev_image_rows_ = points3d_->rows;
      prev_image_cols_ = points3d_->cols;
      normal_computer_ = cv::RgbdNormals(points3d_->rows, points3d_->cols,
                                         CV_32F, *K_, 5, cv::RgbdNormals::RGBD_NORMALS_METHOD_FALS);
    }

    // Compute the normals
    cv::Mat normals;
    normal_computer_(*points3d_, normals);
    std::vector<cv::Mat> channels;
    cv::split(normals, channels);
    cv::Mat channel_view;
    cv::Mat(cv::abs(channels[2])).convertTo(channel_view, CV_8U, 255);

    // Compute the planes
    std::vector<cv::Vec4f> plane_coefficients;
    cv::RgbdPlane plane_finder;
    plane_finder.set("threshold", *plane_threshold_);
    plane_finder.set("min_size", int(*min_table_size_));
    plane_finder.set("sensor_error_a", 0.0075);
    plane_finder(*points3d_, normals, *table_mask_, plane_coefficients);

    // Figure out the points of each plane
    std::vector<std::vector<cv::Point2i> > points_for_hull(
      plane_coefficients.size());
    cv::Mat_<cv::Vec3f>::const_iterator point3d = points3d_->begin<cv::Vec3f>();
    cv::Mat_<uchar>::const_iterator point_mask = table_mask_->begin<uchar>();
    cv::Point2i prev_point;
    for (int y = 0; y < table_mask_->rows; ++y) {
      int prev_index = 255;
      for (int x = 0; x < table_mask_->cols; ++x, ++point3d, ++point_mask) {
        int index = *point_mask;
        if (index == 255) {
          // Close the previous segment
          if (prev_index != 255)
            points_for_hull[prev_index].push_back(prev_point);
          prev_index = 255;
          continue;
        }
        // Add it to the points to compute the hull only if it is different for the previous one
        // or if it is the first/last one on a line
        if (index != prev_index) {
          points_for_hull[index].push_back(cv::Point2i(x, y));
          if (prev_index != 255)
            points_for_hull[prev_index].push_back(prev_point);
        }
        prev_index = index;
        prev_point = cv::Point2i(x, y);
      }
      // Add the last point of the line if it belongs to a plane
      if (prev_index != 255)
        points_for_hull[prev_index].push_back(prev_point);
    }

    // Fill the outputs
    clouds_hull_->clear();
    table_coefficients_->clear();
    for (int i = 0; i < points_for_hull.size(); ++i) {
      // Compute the convex hull
      std::vector<cv::Point2i> hull;
      cv::convexHull(points_for_hull[i], hull);

      // Add the plane coefficients but make sure the normal points towards the camera
      if (plane_coefficients[i][2] < 0)
        table_coefficients_->push_back(plane_coefficients[i]);
      else
        table_coefficients_->push_back(-plane_coefficients[i]);

      // Add the point cloud
      std::vector<cv::Vec3f> out;
      out.reserve(hull.size());
      BOOST_FOREACH(const cv::Point2i & point2d, hull) {
        out.push_back((*points3d_).at<cv::Vec3f>(point2d.y, point2d.x));
      }
      clouds_hull_->push_back(out);
    }

    return ecto::OK;
  }
  private:
    /** The minimum number of points deemed necessary to find a table */
    ecto::spore<size_t> min_table_size_;
    /** The distance used as a threshold when finding a plane */
    ecto::spore<float> plane_threshold_;

    /** The input calibration matrix */
    ecto::spore<cv::Mat> points3d_;
    /** The input cloud */
    ecto::spore<cv::Mat> K_;
    /** The mask of the foundplanes */
    ecto::spore<cv::Mat> table_mask_;
    /** The output cloud */
    ecto::spore<std::vector<std::vector<cv::Vec3f> > > clouds_hull_;
    /** The coefficients of the tables */
    ecto::spore<std::vector<cv::Vec4f> > table_coefficients_;
    /** The frame id of the vertical direction */
    ecto::spore<std::string> up_frame_id_;

    ecto::spore<float> table_cluster_tolerance_;

    /** Cache the size of the previous image */
    int prev_image_rows_, prev_image_cols_;
    /** Cache the normal computer as it precomputes data */
    cv::RgbdNormals normal_computer_;
  };
}

ECTO_CELL(tabletop_table, tabletop::TableDetector, "TableDetector", "Given a point cloud, find  a potential table.");
