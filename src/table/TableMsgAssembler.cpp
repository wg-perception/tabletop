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

#include <opencv2/core/core.hpp>

#include <fstream>
#include <iostream>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <ecto/ecto.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>

#include <object_recognition_msgs/Table.h>
#include <object_recognition_msgs/TableArray.h>

#include <tabletop_object_detector/marker_generator.h>
#include <object_recognition_core/common/pose_result.h>

using object_recognition_core::common::PoseResult;

using ecto::tendrils;

/** Ecto implementation of a module that takes
 *
 */
struct TableMsgAssembler {
  static void
  declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs) {
    inputs.declare(&TableMsgAssembler::image_message_, "image_message", "the image message to get the header").required(
      true);
    inputs.declare(&TableMsgAssembler::pose_results_, "pose_results", "The results of object recognition").required(
      true);
    inputs.declare(&TableMsgAssembler::clouds_hull_, "clouds_hull", "The hull of the samples.").required(true);

    outputs.declare<object_recognition_msgs::TableArrayConstPtr>("table_array_msg", "The message for the found tables");
  }

  /** Get the 2d keypoints and figure out their 3D position from the depth map
   * @param inputs
   * @param outputs
   * @return
   */
  int
  process(const tendrils& inputs, const tendrils& outputs) {
    typedef pcl::PointXYZ Point;

    std::string frame_id;
    if (*image_message_)
      frame_id = (*image_message_)->header.frame_id;

    object_recognition_msgs::TableArray table_array_msg;

    std_msgs::Header message_header;
    message_header.frame_id = frame_id;

    for (size_t table_index = 0; table_index < pose_results_->size(); ++table_index) {
      const PoseResult& pose_result = (*pose_results_)[table_index];
      pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_hull = (*clouds_hull_)[table_index];

      cv::Matx33f R = pose_result.R<cv::Mat_<float> >();
      cv::Vec3f T = pose_result.T<cv::Mat_<float> >();

      object_recognition_msgs::Table table = getTable<pcl::PointCloud<pcl::PointXYZ> >(message_header, R, T, *cloud_hull);

      // ---[ Add the convex hull as a triangle mesh to the Table message
      addConvexHullTable<pcl::PointCloud<pcl::PointXYZ> >(table, *cloud_hull, R, T);
      table_array_msg.tables.push_back(table);
    }

    outputs["table_array_msg"] << object_recognition_msgs::TableArrayConstPtr(new object_recognition_msgs::TableArray(table_array_msg));

    return ecto::OK;
  }
private:
  template<class PointCloudType>
  void
  addConvexHullTable(object_recognition_msgs::Table& table, const PointCloudType& convex_hull, const cv::Matx33f& R, const cv::Vec3f& T) {
    //create a triangle mesh out of the convex hull points and add it to the table message
    //Make sure the points belong to the table frame
    for (size_t i = 0; i < convex_hull.points.size(); i++) {
      geometry_msgs::Point vertex;
      cv::Vec3f point = R.t() * cv::Vec3f(convex_hull.points[i].x, convex_hull.points[i].y, convex_hull.points[i].z) - R.t() * T;
      vertex.x = point[0];
      vertex.y = point[1];
      vertex.z = point[2];
      table.convex_hull.vertices.push_back(vertex);

      if (i == 0 || i == convex_hull.points.size() - 1)
        continue;

      shape_msgs::MeshTriangle tri;
      tri.vertex_indices[0] = 0;
      tri.vertex_indices[1] = i;
      tri.vertex_indices[2] = i + 1;
      table.convex_hull.triangles.push_back(tri);
    }
  }

  template<class PointCloudType>
  object_recognition_msgs::Table
  getTable(const std_msgs::Header& cloud_header, const cv::Matx33f& R, const cv::Vec3f& T,
           const PointCloudType& convex_hull) {
    object_recognition_msgs::Table table;

    //get the extents of the table
    table.x_min = std::numeric_limits<float>::max();
    table.x_max = std::numeric_limits<float>::min();
    table.y_min = std::numeric_limits<float>::max();
    table.y_max = std::numeric_limits<float>::min();

    for (size_t i = 0; i < convex_hull.points.size(); ++i) {
      cv::Vec3f point = R.t() * cv::Vec3f(convex_hull.points[i].x, convex_hull.points[i].y, convex_hull.points[i].z) - R.t() * T;
      if (point[0] < table.x_min && point[0] > -3.0)
        table.x_min = point[0];
      if (point[0] > table.x_max && point[0] < 3.0)
        table.x_max = point[0];
      if (point[1] < table.y_min && point[1] > -3.0)
        table.y_min = point[1];
      if (point[1] > table.y_max && point[1] < 3.0)
        table.y_max = point[1];
    }

    geometry_msgs::Pose table_pose;
    table_pose.position.x = T[0];
    table_pose.position.y = T[1];
    table_pose.position.z = T[2];
    Eigen::Matrix3f rotation;
    rotation << R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2);
    Eigen::Quaternionf quaternion(rotation);
    table_pose.orientation.w = quaternion.w();
    table_pose.orientation.x = quaternion.x();
    table_pose.orientation.y = quaternion.y();
    table_pose.orientation.z = quaternion.z();

    table.pose.pose = table_pose;
    table.pose.header = cloud_header;

    return table;
  }

  /** flag indicating whether we run in debug mode */
  ecto::spore<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> > clouds_hull_;

  //! The current marker being published
  ecto::spore<sensor_msgs::ImageConstPtr> image_message_;

  ecto::spore<std::vector<PoseResult> > pose_results_;

  ecto::spore<object_recognition_msgs::TableArrayConstPtr> table_array_msg_;
};

ECTO_CELL(tabletop_table, TableMsgAssembler, "TableMsgAssembler",
          "Given a point cloud, find  a potential table.");
