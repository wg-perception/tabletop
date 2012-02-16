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

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <tabletop/Table.h>
#include <tabletop/table/tabletop_segmenter.h>

#include "tabletop_object_detector/marker_generator.h"
#include <object_recognition/common/pose_result.h>

using object_recognition::common::PoseResult;

using ecto::tendrils;

namespace tabletop
{
  /** Ecto implementation of a module that takes
   *
   */
  struct TableMsgAssembler
  {
    static void
    declare_params(ecto::tendrils& params)
    {
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&TableMsgAssembler::clusters_, "clusters", "The clusters on top of the table.").required(true);
      inputs.declare(&TableMsgAssembler::image_message_, "image_message", "the image message to get the header").required(
          true);
      inputs.declare(&TableMsgAssembler::pose_results_, "pose_results", "The results of object recognition").required(
          true);
      inputs.declare(&TableMsgAssembler::table_projected_ptr_, "cloud", "Some samples from the table.").required(true);
      inputs.declare(&TableMsgAssembler::table_hull_ptr_, "cloud_hull", "The hull of the samples.").required(true);

      outputs.declare(&TableMsgAssembler::marker_array_delete_, "marker_array_delete", "The markers to delete");
      outputs.declare(&TableMsgAssembler::marker_array_clusters_, "marker_array_clusters",
                      "The markers of the clusters");
      outputs.declare(&TableMsgAssembler::marker_hull_, "marker_hull", "The marker for the table hull");
      outputs.declare(&TableMsgAssembler::marker_origin_, "marker_origin", "The marker for the origin of the table");
      outputs.declare(&TableMsgAssembler::marker_table_, "marker_table", "The marker for the table");
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      current_marker_id_ = 0;
      num_markers_published_ = 0;
      flatten_table_ = false;
    }

    /** Get the 2d keypoints and figure out their 3D position from the depth map
     * @param inputs
     * @param outputs
     * @return
     */
    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      typedef pcl::PointXYZ Point;

      std::string frame_id;
      if (*image_message_)
        frame_id = (*image_message_)->header.frame_id;
      // Delete the old markers
      clearOldMarkers(frame_id);

      std_msgs::Header message_header;
      message_header.frame_id = frame_id;

      BOOST_FOREACH(const PoseResult & pose_result, *pose_results_)
          {
            sensor_msgs::PointCloud table_points;
            sensor_msgs::PointCloud table_hull_points;
            tf::Transform table_plane_trans = getPlaneTransform(pose_result);

            Table table;

            table_points.header.frame_id = frame_id;
            table_hull_points.header.frame_id = frame_id;
            (*table_projected_ptr_)->header.frame_id = frame_id;
            (*table_hull_ptr_)->header.frame_id = frame_id;

            if (!flatten_table_)
            {
              // --- [ Take the points projected on the table and transform them into the PointCloud message
              //  while also transforming them into the table's coordinate system
              if (!getPlanePoints<Point>(*(*table_projected_ptr_), table_plane_trans, table_points))
              {
                //response.result = response.OTHER_ERROR;
                return ecto::OK;
              }

              // ---[ Create the table message
              // TODO use the original cloud header
              table = getTable<sensor_msgs::PointCloud>(message_header, table_plane_trans, table_points);

              // ---[ Convert the convex hull points to table frame
              if (!getPlanePoints<Point>(*(*table_hull_ptr_), table_plane_trans, table_hull_points))
              {
                //response.result = response.OTHER_ERROR;
                return ecto::OK;
              }
            }
            if (flatten_table_)
            {
              // if flattening the table, find the center of the convex hull and move the table frame there
              tf::Vector3 flat_table_pos;
              double avg_x, avg_y, avg_z;
              avg_x = avg_y = avg_z = 0;
              for (size_t i = 0; i < (*table_projected_ptr_)->points.size(); i++)
              {
                avg_x += (*table_projected_ptr_)->points[i].x;
                avg_y += (*table_projected_ptr_)->points[i].y;
                avg_z += (*table_projected_ptr_)->points[i].z;
              }
              avg_x /= (*table_projected_ptr_)->points.size();
              avg_y /= (*table_projected_ptr_)->points.size();
              avg_z /= (*table_projected_ptr_)->points.size();

              // place the new table frame in the center of the convex hull
              flat_table_pos[0] = avg_x;
              flat_table_pos[1] = avg_y;
              flat_table_pos[2] = avg_z;
              table_plane_trans.setOrigin(flat_table_pos);

              // --- [ Take the points projected on the table and transform them into the PointCloud message
              //  while also transforming them into the flat table's coordinate system
              sensor_msgs::PointCloud flat_table_points;
              if (!getPlanePoints<Point>(**table_projected_ptr_, table_plane_trans, flat_table_points))
              {
                //TODOresponse.result = response.OTHER_ERROR;
                return ecto::OK;
              }

              // ---[ Create the table message
              // TODO use the original cloud header
              table = getTable<sensor_msgs::PointCloud>(message_header, table_plane_trans, flat_table_points);

              // ---[ Convert the convex hull points to flat table frame
              if (!getPlanePoints<Point>(**table_hull_ptr_, table_plane_trans, table_hull_points))
              {
                return ecto::OK;
              }
            }

            // ---[ Add the convex hull as a triangle mesh to the Table message
            addConvexHullTable<sensor_msgs::PointCloud>(table, table_hull_points, flatten_table_);
          }

      // Publish each clusters
      addClusterMarkers(*clusters_, message_header);

      return ecto::OK;
    }
  private:
    void
    clearOldMarkers(std::string frame_id)
    {
      visualization_msgs::MarkerArrayPtr marker_array_delete(new visualization_msgs::MarkerArray);

      for (size_t id = current_marker_id_; id < num_markers_published_; id++)
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

    template<typename PointT>
    bool
    getPlanePoints(const pcl::PointCloud<PointT> &table, const tf::Transform& table_plane_trans,
                   sensor_msgs::PointCloud &table_points)
    {
      // Prepare the output
      table_points.header = table.header;
      table_points.points.resize(table.points.size());
      for (size_t i = 0; i < table.points.size(); ++i)
      {
        table_points.points[i].x = table.points[i].x;
        table_points.points[i].y = table.points[i].y;
        table_points.points[i].z = table.points[i].z;
      }

      // Transform the data
      tf::TransformListener listener;
      tf::StampedTransform table_pose_frame(table_plane_trans, table.header.stamp, table.header.frame_id,
                                            "table_frame");
      listener.setTransform(table_pose_frame);
      std::string error_msg;
      if (!listener.canTransform("table_frame", table_points.header.frame_id, table_points.header.stamp, &error_msg))
      {
        return false;
      }
      try
      {
        listener.transformPointCloud("table_frame", table_points, table_points);
      } catch (tf::TransformException &ex)
      {
        return false;
      }
      table_points.header.stamp = table.header.stamp;
      table_points.header.frame_id = "table_frame";
      return true;
    }

    template<class PointCloudType>
    void
    addConvexHullTable(Table &table, const PointCloudType &convex_hull, bool flatten_table)
    {
      //create a triangle mesh out of the convex hull points and add it to the table message
      table.convex_hull.type = table.convex_hull.MESH;
      for (size_t i = 0; i < convex_hull.points.size(); i++)
      {
        geometry_msgs::Point vertex;
        vertex.x = convex_hull.points[i].x;
        vertex.y = convex_hull.points[i].y;
        if (flatten_table)
          vertex.z = 0;
        else
          vertex.z = convex_hull.points[i].z;
        table.convex_hull.vertices.push_back(vertex);

        if (i == 0 || i == convex_hull.points.size() - 1)
          continue;
        table.convex_hull.triangles.push_back(0);
        table.convex_hull.triangles.push_back(i);
        table.convex_hull.triangles.push_back(i + 1);
      }
      visualization_msgs::MarkerPtr marker_hull(new visualization_msgs::Marker);
      *marker_hull = tabletop_object_detector::MarkerGenerator::getConvexHullTableMarker(table.convex_hull);
      marker_hull->header = table.pose.header;
      marker_hull->pose = table.pose.pose;
      marker_hull->ns = "tabletop_node";
      marker_hull->id = current_marker_id_++;
      *marker_hull_ = marker_hull;

      visualization_msgs::MarkerPtr marker_origin(new visualization_msgs::Marker);
      *marker_origin = tabletop_object_detector::MarkerGenerator::createMarker(table.pose.header.frame_id, 0, .0025,
                                                                               .0025, .01, 0, 1, 1,
                                                                               visualization_msgs::Marker::CUBE,
                                                                               current_marker_id_++, "tabletop_node",
                                                                               table.pose.pose);
      *marker_origin_ = marker_origin;
    }

    template<class PointCloudType>
    Table
    getTable(std_msgs::Header cloud_header, const tf::Transform &table_plane_trans, const PointCloudType &table_points)
    {
      Table table;

      //get the extents of the table
      if (!table_points.points.empty())
      {
        table.x_min = table_points.points[0].x;
        table.x_max = table_points.points[0].x;
        table.y_min = table_points.points[0].y;
        table.y_max = table_points.points[0].y;
      }
      for (size_t i = 1; i < table_points.points.size(); ++i)
      {
        if (table_points.points[i].x < table.x_min && table_points.points[i].x > -3.0)
          table.x_min = table_points.points[i].x;
        if (table_points.points[i].x > table.x_max && table_points.points[i].x < 3.0)
          table.x_max = table_points.points[i].x;
        if (table_points.points[i].y < table.y_min && table_points.points[i].y > -3.0)
          table.y_min = table_points.points[i].y;
        if (table_points.points[i].y > table.y_max && table_points.points[i].y < 3.0)
          table.y_max = table_points.points[i].y;
      }

      geometry_msgs::Pose table_pose;
      tf::poseTFToMsg(table_plane_trans, table_pose);
      table.pose.pose = table_pose;
      table.pose.header = cloud_header;

      visualization_msgs::MarkerPtr marker_table(new visualization_msgs::Marker);
      *marker_table = tabletop_object_detector::MarkerGenerator::getTableMarker(table.x_min, table.x_max, table.y_min,
                                                                                table.y_max);
      marker_table->header = cloud_header;
      marker_table->pose = table_pose;
      marker_table->ns = "tabletop_node";
      marker_table->id = current_marker_id_++;
      *marker_table_ = marker_table;

      return table;
    }

    void
    addClusterMarkers(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters, std_msgs::Header cloud_header)
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

    /** The distance used as a threshold when finding a plane */
    bool flatten_table_;

    /** flag indicating whether we run in debug mode */
    ecto::spore<pcl::PointCloud<pcl::PointXYZ>::Ptr> table_projected_ptr_, table_hull_ptr_;
    /** The vertical direction */
    ecto::spore<Eigen::Vector3f> up_direction_;

    //! The current marker being published
    size_t current_marker_id_;
    ecto::spore<sensor_msgs::ImageConstPtr> image_message_;

    ecto::spore<std::vector<PoseResult> > pose_results_;

    ecto::spore<visualization_msgs::MarkerConstPtr> marker_table_;
    ecto::spore<visualization_msgs::MarkerConstPtr> marker_origin_;
    ecto::spore<visualization_msgs::MarkerConstPtr> marker_hull_;
    /** The markers to delete before showing new ones */
    ecto::spore<visualization_msgs::MarkerArrayConstPtr> marker_array_delete_;
    /** The markers of the clusters */
    ecto::spore<visualization_msgs::MarkerArrayConstPtr> marker_array_clusters_;
    /** Number of markers published */
    size_t num_markers_published_;
    /** The point clusters */
    ecto::spore<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> > clusters_;
  };

}

ECTO_CELL(tabletop_table, tabletop::TableMsgAssembler, "TableMsgAssembler",
          "Given a point cloud, find  a potential table.");
