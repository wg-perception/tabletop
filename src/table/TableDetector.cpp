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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/rgbd/rgbd.hpp>

#include <tf/transform_listener.h>

#include <tabletop/table/tabletop_segmenter.h>

using ecto::tendrils;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> Cloud;

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
      params.declare(&TableDetector::normal_k_search_, "normal_k_search",
                     "The number of nearest neighbors to use when computing normals", 10);
      params.declare(&TableDetector::plane_threshold_, "plane_threshold",
                     "The distance used as a threshold when finding a plane", 0.1);
      params.declare(&TableDetector::table_cluster_tolerance_, "table_cluster_tolerance",
                     "The distance used when clustering a plane", 0.2);
      Eigen::Vector3f default_up(0, 0, 1);
//      params.declare(&TableDetector::up_direction_, "vertical_direction", "The vertical direction", default_up);
      params.declare(&TableDetector::up_frame_id_, "vertical_frame_id", "The vertical frame id", "/map");
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&TableDetector::points3d_, "points3d", "The 3dpoints as a cv::Mat_<cv::Vec3f>");
      inputs.declare(&TableDetector::K_, "K", "The calibration matrix");
      inputs.declare(&TableDetector::flatten_plane_, "flatten_plane",
                     "If true, the plane's normal is vertical_direction.", false);

      outputs.declare(&TableDetector::table_coefficients_, "table_coefficients", "The coefficients of planar surfaces.");
      outputs.declare(&TableDetector::table_mask_, "table_mask", "The mask of planar surfaces.");
      outputs.declare(&TableDetector::table_rotations_, "rotations", "The pose rotations of the tables.");
      outputs.declare(&TableDetector::table_translations_, "translations", "The pose translations of the tables");
      outputs.declare(&TableDetector::clouds_out_, "clouds", "Samples that belong to the table.");
      outputs.declare(&TableDetector::clouds_hull_, "clouds_hull", "Hulls of the samples.");
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
    	up_direction_ = Eigen::Vector3f(0,0,1);
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
    cv::Mat normals = normal_computer_(*points3d_);
    std::vector<cv::Mat> channels;
    cv::split(normals, channels);
    cv::Mat channel_view;
    cv::Mat(cv::abs(channels[2])).convertTo(channel_view, CV_8U, 255);

    // Compute the planes
    std::vector<cv::Vec4f> plane_coefficients;
    cv::RgbdPlane plane_finder;
    plane_finder.set("threshold", 0.02);
    plane_finder.set("min_size", int(*min_table_size_));
    plane_finder.set("sensor_error_a", 0.0075);
    plane_finder(*points3d_, normals, *table_mask_, plane_coefficients);

    // Prepare the plane clusters
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_out;
    BOOST_FOREACH(const cv::Vec4f & plane_coefficient, plane_coefficients) {
      pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
      clouds_out.push_back(out);
    }

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
        // Add the point to the plane no matter what
        const cv::Vec3f& point = *point3d;
        clouds_out[index]->push_back(PointT(point[0], point[1], point[2]));
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
    clouds_out_->clear();
    clouds_hull_->clear();
    table_coefficients_->clear();
    for (int i = 0; i < points_for_hull.size(); ++i) {
      // Copy the points out
      clouds_out_->push_back(clouds_out[i]);

      // Compute the convex hull
      std::vector<cv::Point2i> hull;
      cv::convexHull(points_for_hull[i], hull);

      // Add the plane coefficients
      table_coefficients_->push_back(
        Eigen::Vector4f(plane_coefficients[i][0], plane_coefficients[i][1],
                        plane_coefficients[i][2], plane_coefficients[i][3]));

      // Add the point cloud
      pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
      BOOST_FOREACH(const cv::Point2i & point2d, hull) {
        const cv::Vec3f& point3d = (*points3d_).at<cv::Vec3f>(point2d.y, point2d.x);
        out->push_back(PointT(point3d[0], point3d[1], point3d[2]));
      }
      clouds_hull_->push_back(out);
    }

    // Compute the corresponding poses
    table_rotations_->resize(table_coefficients_->size());
    table_translations_->resize(table_coefficients_->size());
    for (size_t i = 0; i < table_coefficients_->size(); ++i)
      getPlaneTransform((*table_coefficients_)[i], up_direction_, *flatten_plane_, (*table_translations_)[i],
                        (*table_rotations_)[i]);

    return ecto::OK;
  }
  private:
    /** The minimum number of points deemed necessary to find a table */
    ecto::spore<size_t> min_table_size_;
    /** The number of nearest neighbors to use when computing normals */
    ecto::spore<unsigned int> normal_k_search_;
    /** The distance used as a threshold when finding a plane */
    ecto::spore<float> plane_threshold_;

    /** if true, the plane coefficients are modified so that up_direction_in is the normal */
    ecto::spore<bool> flatten_plane_;
    /** The input calibration matrix */
    ecto::spore<cv::Mat> points3d_;
    /** The input cloud */
    ecto::spore<cv::Mat> K_;
    /** The mask of the foundplanes */
    ecto::spore<cv::Mat> table_mask_;
    /** The input cloud */
    ecto::spore<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> > clouds_out_;
    /** The output cloud */
    ecto::spore<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> > clouds_hull_;
    /** The rotations of the tables */
    ecto::spore<std::vector<Eigen::Matrix3f> > table_rotations_;
    /** The translations of the tables */
    ecto::spore<std::vector<Eigen::Vector3f> > table_translations_;
    /** The minimum number of inliers in order to do pose matching */
    ecto::spore<std::vector<Eigen::Vector4f> > table_coefficients_;
    /** The vertical direction */
//    ecto::spore<Eigen::Vector3f> up_direction_;
    Eigen::Vector3f up_direction_;
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
