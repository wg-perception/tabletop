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

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <ecto/ecto.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

#include <arm_navigation_msgs/Shape.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <object_recognition_msgs/shape_conversion.h>
#include <object_recognition_msgs/Table.h>
#include <object_recognition_msgs/TableArray.h>
#include <tabletop/table/tabletop_segmenter.h>

#include "tabletop_object_detector/marker_generator.h"
#include <object_recognition_core/common/pose_result.h>

#include <tf/LinearMath/Transform.h>

using object_recognition_core::common::PoseResult;

using ecto::tendrils;

namespace tabletop
{
  /** Ecto implementation of a module that takes
   *
   */
  struct TableVisualizationMsgAssembler
  {
    static void
    declare_params(ecto::tendrils& params)
    {
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&TableVisualizationMsgAssembler::clusters_, "clusters", "The clusters on top of the table.").required(
          true);
      inputs.declare(&TableVisualizationMsgAssembler::image_message_, "image_message",
                     "the image message to get the header").required(true);
      inputs.declare(&TableVisualizationMsgAssembler::pose_results_, "pose_results",
                     "The results of object recognition").required(true);
      inputs.declare(&TableVisualizationMsgAssembler::table_array_msg_, "table_array_msg",
                     "The message for the found tables").required(true);

      outputs.declare(&TableVisualizationMsgAssembler::marker_array_delete_, "marker_array_delete",
                      "The markers to delete");
      outputs.declare(&TableVisualizationMsgAssembler::marker_array_clusters_, "marker_array_clusters",
                      "The markers of the clusters");
      outputs.declare<object_recognition_msgs::TableArrayConstPtr>("table_array_msg",
                                                                   "The message for the found tables");
      outputs.declare<visualization_msgs::MarkerArrayConstPtr>("marker_array_hull", "The marker for the table hull");
      outputs.declare<visualization_msgs::MarkerArrayConstPtr>("marker_array_origin",
                                                               "The marker for the origin of the table");
      outputs.declare<visualization_msgs::MarkerArrayConstPtr>("marker_array_table", "The marker for the table");
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      current_marker_id_ = 0;
      num_markers_published_ = 0;
    }

    /** Get the 2d keypoints and figure out their 3D position from the depth map
     * @param inputs
     * @param outputs
     * @return
     */
    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      std::string frame_id;
      if (*image_message_)
        frame_id = (*image_message_)->header.frame_id;
      // Delete the old markers
      clearOldMarkers(frame_id);

      marker_array_table_.markers.clear();
      marker_array_table_.markers.reserve(pose_results_->size());
      marker_array_origin_.markers.clear();
      marker_array_origin_.markers.reserve(pose_results_->size());
      marker_array_hull_.markers.clear();
      marker_array_hull_.markers.reserve(pose_results_->size());

      std_msgs::Header message_header;
      message_header.frame_id = frame_id;

      for (size_t table_index = 0; table_index < pose_results_->size(); ++table_index)
      {
        const PoseResult & pose_result = (*pose_results_)[table_index];

        const object_recognition_msgs::Table & table = (*table_array_msg_)->tables[table_index];

        getTable<sensor_msgs::PointCloud>(table, message_header);

        // ---[ Add the convex hull as a triangle mesh to the Table message
        addConvexHullTable<sensor_msgs::PointCloud>(table);

        // Publish each clusters
        addClusterMarkers((*clusters_)[table_index], table.pose.header/*message_header*/);
      }

      outputs["marker_array_hull"]
      << visualization_msgs::MarkerArrayConstPtr(new visualization_msgs::MarkerArray(marker_array_hull_));
      outputs["marker_array_origin"]
      << visualization_msgs::MarkerArrayConstPtr(new visualization_msgs::MarkerArray(marker_array_origin_));
      outputs["marker_array_table"]
      << visualization_msgs::MarkerArrayConstPtr(new visualization_msgs::MarkerArray(marker_array_table_));

      return ecto::OK;
    }
  private:
    void
    clearOldMarkers(const std::string & frame_id)
    {
      visualization_msgs::MarkerArrayPtr marker_array_delete(new visualization_msgs::MarkerArray);

      for (size_t id = 0; id < num_markers_published_; id++)
      {
        visualization_msgs::Marker marker_delete;
        marker_delete.header.stamp = ros::Time::now();
        marker_delete.header.frame_id = frame_id;
        marker_delete.id = id;
        marker_delete.action = visualization_msgs::Marker::DELETE;
        marker_delete.ns = "tabletop_node";
        marker_array_delete->markers.push_back(marker_delete);
      }
      num_markers_published_ = current_marker_id_;
      current_marker_id_ = 0;

      *marker_array_delete_ = marker_array_delete;
    }

    /*! Assumes plane coefficients are of the form ax+by+cz+d=0, normalized */
    tf::Transform
    getPlaneTransform(const PoseResult & pose_result)
    {
      Eigen::Vector3f translation = pose_result.T<Eigen::Vector3f>();
      Eigen::Matrix3f rotation = pose_result.R<Eigen::Matrix3f>();

      tf::Vector3 position_tf(translation[0], translation[1], translation[2]);
      tf::Matrix3x3 rotation_tf(rotation.coeff(0, 0), rotation.coeff(0, 1), rotation.coeff(0, 2), rotation.coeff(1, 0),
                                rotation.coeff(1, 1), rotation.coeff(1, 2), rotation.coeff(2, 0), rotation.coeff(2, 1),
                                rotation.coeff(2, 2));

      tf::Quaternion orientation;
      rotation_tf.getRotation(orientation);

      return tf::Transform(orientation, position_tf);
    }

    template<class PointCloudType>
    void
    addConvexHullTable(const object_recognition_msgs::Table &table)
    {
      //create a triangle mesh out of the convex hull points and add it to the table message
      visualization_msgs::MarkerPtr marker_hull(new visualization_msgs::Marker);
      arm_navigation_msgs::Shape arm_nav_convex_hull = mesh_to_an_shape(table.convex_hull);

      *marker_hull = tabletop_object_detector::MarkerGenerator::getConvexHullTableMarker(arm_nav_convex_hull);
      marker_hull->header = table.pose.header;
      marker_hull->pose = table.pose.pose;
      marker_hull->ns = "tabletop_node";
      marker_hull->id = current_marker_id_++;
      marker_array_hull_.markers.push_back(*marker_hull);

      // Deal with the marker for the origin of the table
      visualization_msgs::MarkerPtr marker_origin(new visualization_msgs::Marker);
      Eigen::Matrix3f rotation_table(
          Eigen::Quaternionf(table.pose.pose.orientation.w, table.pose.pose.orientation.x,
                             table.pose.pose.orientation.y, table.pose.pose.orientation.z));

      Eigen::Matrix3f rotation_arrow;
      rotation_arrow << 0, 0, -1, 0, 1, 0, 1, 0, 0;
      Eigen::Quaternionf quat(rotation_table * rotation_arrow);

      ::geometry_msgs::Pose_<std::allocator<void> > marker_pose = table.pose.pose;
      marker_pose.orientation.w = quat.w();
      marker_pose.orientation.x = quat.x();
      marker_pose.orientation.y = quat.y();
      marker_pose.orientation.z = quat.z();
      *marker_origin = tabletop_object_detector::MarkerGenerator::createMarker(table.pose.header.frame_id, 10, .2, .2,
                                                                               .05, 0, 1, 1,
                                                                               visualization_msgs::Marker::ARROW,
                                                                               current_marker_id_++, "tabletop_node",
                                                                               marker_pose);
      marker_array_origin_.markers.push_back(*marker_origin);
    }

    template<class PointCloudType>
    void
    getTable(const object_recognition_msgs::Table & table, const std_msgs::Header &cloud_header)
    {
      visualization_msgs::Marker marker_table = tabletop_object_detector::MarkerGenerator::getTableMarker(table.x_min,
                                                                                                          table.x_max,
                                                                                                          table.y_min,
                                                                                                          table.y_max);
      marker_table.header = cloud_header;
      marker_table.pose = table.pose.pose;
      marker_table.ns = "tabletop_node";
      marker_table.id = current_marker_id_++;
      marker_array_table_.markers.push_back(marker_table);
    }

    void
    addClusterMarkers(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters,
                      const std_msgs::Header &cloud_header)
    {
      visualization_msgs::MarkerArrayPtr marker_array_clusters(new visualization_msgs::MarkerArray);

      for (size_t i = 0; i < clusters.size(); i++)
      {
        visualization_msgs::Marker cloud_marker = tabletop_object_detector::MarkerGenerator::getCloudMarker(
            *(clusters[i]));
        cloud_marker.header = cloud_header;
        cloud_marker.pose.orientation.w = 1;
        cloud_marker.ns = "tabletop_node";
        cloud_marker.id = current_marker_id_++;
        marker_array_clusters->markers.push_back(cloud_marker);
      }

      *marker_array_clusters_ = marker_array_clusters;
    }

    //! The current marker being published
    size_t current_marker_id_;
    ecto::spore<sensor_msgs::ImageConstPtr> image_message_;

    ecto::spore<std::vector<PoseResult> > pose_results_;

    ecto::spore<object_recognition_msgs::TableArrayConstPtr> table_array_msg_;

    visualization_msgs::MarkerArray marker_array_table_;
    visualization_msgs::MarkerArray marker_array_origin_;
    visualization_msgs::MarkerArray marker_array_hull_;
    /** The markers to delete before showing new ones */
    ecto::spore<visualization_msgs::MarkerArrayConstPtr> marker_array_delete_;
    /** The markers of the clusters */
    ecto::spore<visualization_msgs::MarkerArrayConstPtr> marker_array_clusters_;
    /** Number of markers published */
    size_t num_markers_published_;
    /** For each table, a vector of clusters */
    ecto::spore<std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> > > clusters_;
  };
}

ECTO_CELL(tabletop_table, tabletop::TableVisualizationMsgAssembler, "TableVisualizationMsgAssembler",
          "Given a point cloud, find  a potential table.");
