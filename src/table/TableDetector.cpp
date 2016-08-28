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
#if CV_MAJOR_VERSION >= 3
#include <opencv2/rgbd.hpp>
namespace cv {
  using namespace rgbd;
}
#else
#include <opencv2/rgbd/rgbd.hpp>
#endif
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <object_recognition_core/common/pose_result.h>

using object_recognition_core::common::PoseResult;
using ecto::tendrils;

/**
 * If the equation of the plane is ax+by+cz+d=0, the pose (R,t) is such that it takes the horizontal plane (z=0)
 * to the current equation
 */
void
getPlaneTransform(const cv::Vec4f& plane_coefficients, cv::Matx33f& rotation, cv::Vec3f& translation)
{
  double a = plane_coefficients[0], b = plane_coefficients[1], c = plane_coefficients[2], d = plane_coefficients[3];
  // assume plane coefficients are normalized
  translation = cv::Vec3f(-a * d, -b * d, -c * d);
  cv::Vec3f z(a, b, c);

  //try to align the x axis with the x axis of the original frame
  //or the y axis if z and x are too close too each other
  cv::Vec3f x(1, 0, 0);
  if (fabs(z.dot(x)) > 1.0 - 1.0e-4)
    x = cv::Vec3f(0, 1, 0);
  cv::Vec3f y = z.cross(x);
  x = y.cross(z);
  x = x / norm(x);
  y = y / norm(y);

  rotation = cv::Matx33f(x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]);
}

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
      inputs.declare(&TableDetector::points3d_, "points3d", "The 3dpoints as a cv::Mat_<cv::Vec3f>").required();
      inputs.declare(&TableDetector::K_, "K", "The calibration matrix").required();

      outputs.declare(&TableDetector::table_coefficients_, "table_coefficients", "The coefficients of planar surfaces.");
      outputs.declare(&TableDetector::table_mask_, "table_mask", "The mask of planar surfaces.");
      outputs.declare(&TableDetector::clouds_hull_, "clouds_hull", "Hulls of the samples.");
      outputs.declare(&TableDetector::pose_results_, "pose_results", "The results of object recognition");
    }


    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      ros::NodeHandle nh("~");
      nh.param("filter_planes", filter_planes_, false);
      nh.param("min_table_height", min_table_height_, 0.5);
      nh.param("max_table_height", max_table_height_, 1.0);
      nh.param("robot_frame", robot_frame_id_, std::string("/base_link"));
      nh.param("sensor_frame", sensor_frame_id_, std::string("/head_mount_kinect_rgb_optical_frame"));

      double max_angle_diff;
      double table_normal_x;
      double table_normal_y;
      double table_normal_z;
      nh.param("max_angle_diff", max_angle_diff, 0.1);
      nh.param("table_normal_x", table_normal_x, 0.0);
      nh.param("table_normal_y", table_normal_y, 0.0);
      nh.param("table_normal_z", table_normal_z, 1.0);
      tf_.reset (new tf::TransformListener);
      min_angle_cos_ = cos(max_angle_diff);

      axis_ = tf::Vector3 (table_normal_x, table_normal_y, table_normal_z);
      std::cout << __LINE__ << " :: " << min_table_height_ << " , " << max_table_height_ << " , " << min_angle_cos_
                << " , " << robot_frame_id_ << " , " << sensor_frame_id_ << std::endl;
    }

  /** Get the 2d keypoints and figure out their 3D position from the depth map
   * @param inputs
   * @param outputs
   * @return
   */
  int
  process(const tendrils& inputs, const tendrils& outputs)
  {
    clouds_hull_->clear();
    table_coefficients_->clear();
    pose_results_->clear();
    if (!filter_planes_ || tf_->waitForTransform(robot_frame_id_, sensor_frame_id_, ros::Time(0), ros::Duration(0.5)))
    {
      if ((points3d_->rows != prev_image_rows_) || (points3d_->cols != prev_image_cols_))
      {
        prev_image_rows_ = points3d_->rows;
        prev_image_cols_ = points3d_->cols;
        normal_computer_ = cv::RgbdNormals(points3d_->rows, points3d_->cols, CV_32F, *K_, 5, cv::RgbdNormals::RGBD_NORMALS_METHOD_FALS);
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
#if CV_MAJOR_VERSION >= 3
      plane_finder.setThreshold(*plane_threshold_);
      plane_finder.setMinSize(int(*min_table_size_));
      plane_finder.setSensorErrorA(0.0075);
#else
      plane_finder.set("threshold", *plane_threshold_);
      plane_finder.set("min_size", int(*min_table_size_));
      plane_finder.set("sensor_error_a", 0.0075);
#endif
      plane_finder(*points3d_, normals, *table_mask_, plane_coefficients);

      std::vector<bool> valid_planes;
      unsigned valid_plane_count = 0;
      if (filter_planes_) // -> tf_->waitForTransform = true
      {
        valid_planes = std::vector<bool>(plane_coefficients.size(), false);
        tf::StampedTransform transform;
        tf_->lookupTransform(robot_frame_id_, sensor_frame_id_,ros::Time(0), transform);
        tf::Matrix3x3 basis = transform.getBasis();
        tf::Vector3 origin = transform.getOrigin();

        for (unsigned pIdx = 0; pIdx < plane_coefficients.size(); ++pIdx)
        {
          tf::Vector3 normal (plane_coefficients[pIdx][0], plane_coefficients[pIdx][1], plane_coefficients[pIdx][2]);
          double dist = plane_coefficients[pIdx][3];

          tf::Vector3 normal_ = basis * normal;
          double dist_ = normal_.dot (origin) - dist;
          if (normal_.dot(axis_) >= min_angle_cos_ && dist_ >= min_table_height_ && dist_ <= max_table_height_)
          {
            valid_planes [pIdx] = true;
            ++valid_plane_count;
          }
        }
      }
      else
      {
        valid_planes = std::vector<bool>(plane_coefficients.size(), true);
        valid_plane_count = plane_coefficients.size();
      }

      if (valid_plane_count > 0)
      {
        // Figure out the points of each plane
        std::vector<std::vector<cv::Point2i> > points_for_hull(plane_coefficients.size());
        cv::Mat_<cv::Vec3f>::const_iterator point3d = points3d_->begin<cv::Vec3f>();
        cv::Mat_<uchar>::const_iterator point_mask = table_mask_->begin<uchar>();
        cv::Point2i prev_point;
        for (int y = 0; y < table_mask_->rows; ++y)
        {
          int prev_index = 255;
          for (int x = 0; x < table_mask_->cols; ++x, ++point3d, ++point_mask)
          {
            int index = *point_mask;
            if (index == 255)
            {
              // Close the previous segment
              if (prev_index != 255)
                points_for_hull[prev_index].push_back(prev_point);
              prev_index = 255;
              continue;
            }
            // Add it to the points to compute the hull only if it is different for the previous one
            // or if it is the first/last one on a line
            if (index != prev_index)
            {
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
        for (int i = 0; i < points_for_hull.size(); ++i)
        {
          if (valid_planes[i])
          {
            // Compute the convex hull
            std::vector<cv::Point2i> hull;
            cv::convexHull(points_for_hull[i], hull);

            // Add the plane coefficients but make sure the normal points towards the camera
            if (plane_coefficients[i][2] < 0)
              table_coefficients_->push_back(plane_coefficients[i]);
            else
              table_coefficients_->push_back(-plane_coefficients[i]);

            // Compute the transforms
            cv::Matx33f R;
            cv::Vec3f T;
            getPlaneTransform((*table_coefficients_)[i], R, T);
            PoseResult pose_result;
            pose_result.set_R(cv::Mat(R));

            // Get the center of the hull
            cv::Moments m = moments(hull);
            int y = int(m.m01/m.m00), x = int(m.m10/m.m00);
            int y_min = y, y_max = y, x_min = x, x_max = x;
            cv::Vec3f center;
            // Make sure we get a non-NaN
            bool is_found = false;
            while (!is_found) {

              for(x = x_min; x <= x_max && !is_found; ++x) {
                if (is_found = (table_mask_->at<uchar>(y_min, x) == i))
                  center = (*points3d_).at<cv::Vec3f>(y_min, x);
                if (is_found = (table_mask_->at<uchar>(y_max, x) == i))
                  center = (*points3d_).at<cv::Vec3f>(y_max, x);
              }
              for(int y = y_min+1; y <= y_max-1 && !is_found; ++y) {
                if (is_found = (table_mask_->at<uchar>(y, x_min) == i))
                  center = (*points3d_).at<cv::Vec3f>(y, x_min);
                if (is_found = (table_mask_->at<uchar>(y, x_max) == i))
                  center = (*points3d_).at<cv::Vec3f>(y, x_max);
              }
              --x_min;
              --y_min;
              ++x_max;
              ++y_max;
            }
            pose_result.set_T(cv::Mat(center));
            pose_results_->push_back(pose_result);

            // Add the point cloud
            std::vector<cv::Vec3f> out;
            out.reserve(hull.size());
            BOOST_FOREACH(const cv::Point2i & point2d, hull) {
              // Project the point onto the plane
              cv::Vec3f point = (*points3d_).at<cv::Vec3f>(point2d.y, point2d.x);
              double distance = plane_coefficients[i][0] * point[0] + plane_coefficients[i][1] * point[1]
                + plane_coefficients[i][2] * point[2] + plane_coefficients[i][3];
              cv::Vec3f plane_normal(plane_coefficients[i][0], plane_coefficients[i][1], plane_coefficients[i][2]);
              cv::Vec3f projected_point = point - distance * plane_normal;
              out.push_back(projected_point);
            }
            clouds_hull_->push_back(out);
          }
        }
      }
    }
    else
    {
      *table_mask_ = cv::Mat (points3d_->rows, points3d_->cols, CV_8UC1);
      ROS_WARN ("Could not get transformation, skipping frame\n");
    }

    return ecto::OK;
  }
  private:
    /** The minimum number of points deemed necessary to find a table */
    ecto::spore<size_t> min_table_size_;
    /** The distance used as a threshold when finding a plane */
    ecto::spore<float> plane_threshold_;

    /** The input cloud */
    ecto::spore<cv::Mat> points3d_;
    /** The input calibration matrix */
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

    /** The poses of the different planes */
    ecto::spore<std::vector<PoseResult> > pose_results_;

    /** Cache the size of the previous image */
    int prev_image_rows_, prev_image_cols_;
    /** Cache the normal computer as it precomputes data */
    cv::RgbdNormals normal_computer_;

    boost::shared_ptr<tf::TransformListener> tf_;
    double min_table_height_;
    double max_table_height_;
    tf::Vector3 axis_;
    double min_angle_cos_;
    std::string robot_frame_id_;
    std::string sensor_frame_id_;
    bool filter_planes_;
  };
}

ECTO_CELL(tabletop_table, tabletop::TableDetector, "TableDetector", "Given a point cloud, find  a potential table.");
