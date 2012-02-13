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

#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <tabletop/Table.h>
#include <tabletop/table/tabletop_segmenter.h>

using ecto::tendrils;

namespace tabletop
{
  /** Ecto implementation of a module that takes
   *
   */
  struct TablePublisher
  {
    static void
    declare_params(ecto::tendrils& params)
    {
      params.declare(&TablePublisher::table_coefficients_ptr_, "filter_limits",
                     "The limits of the interest box to find a table, in order [xmin,xmax,ymin,ymax,zmin,zmax]");
      params.declare(&TablePublisher::min_cluster_size_, "min_cluster_size",
                     "The minimum number of points deemed necessary to find a table.", 1000);
      params.declare(&TablePublisher::plane_detection_voxel_size_, "plane_detection_voxel_size",
                     "The size of a voxel cell when downsampling ", 0.01);
      params.declare(&TablePublisher::normal_k_search_, "normal_k_search",
                     "The number of nearest neighbors to use when computing normals", 10);
      params.declare(&TablePublisher::up_direction_, "vertical_direction", "The vertical direction");
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&TablePublisher::cloud_, "cloud", "The point cloud in which to find a table.");
      inputs.declare(&TablePublisher::table_projected_ptr_, "cloud_projected", "The projected points on the table.");
      inputs.declare(&TablePublisher::table_hull_ptr_, "cloud_hull", "The projected points on the table.");

      outputs.declare(&TablePublisher::table_inliers_, "inliers",
                      "The indices of the original points belonging to the table.");
      outputs.declare(&TablePublisher::table_coefficients_, "coefficients", "The coefficients of the table.");
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
      typedef pcl::PointXYZ Point;

      sensor_msgs::PointCloud table_points;
      sensor_msgs::PointCloud table_hull_points;
      tf::Transform table_plane_trans = getPlaneTransform(*table_coefficients_ptr_, false);
      tf::Transform table_plane_trans_flat;

      Table table;
      if (!flatten_table_)
      {
        // --- [ Take the points projected on the table and transform them into the PointCloud message
        //  while also transforming them into the table's coordinate system
        if (!getPlanePoints<Point>(*(*table_projected_ptr_), table_plane_trans, table_points))
        {
          //response.result = response.OTHER_ERROR;
          return;
        }

        // ---[ Create the table message
        table = getTable<sensor_msgs::PointCloud>(cloud.header, table_plane_trans, table_points);

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
        table_plane_trans_flat = getPlaneTransform(*table_coefficients_ptr_, flatten_table_);
        tf::Vector3 flat_table_pos;
        double avg_x, avg_y, avg_z;
        avg_x = avg_y = avg_z = 0;
        for (size_t i = 0; i < table_projected_ptr->points.size(); i++)
        {
          avg_x += table_projected_ptr->points[i].x;
          avg_y += table_projected_ptr->points[i].y;
          avg_z += table_projected_ptr->points[i].z;
        }
        avg_x /= table_projected_ptr->points.size();
        avg_y /= table_projected_ptr->points.size();
        avg_z /= table_projected_ptr->points.size();
        ROS_INFO("average x,y,z = (%.5f, %.5f, %.5f)", avg_x, avg_y, avg_z);

        // place the new table frame in the center of the convex hull
        flat_table_pos[0] = avg_x;
        flat_table_pos[1] = avg_y;
        flat_table_pos[2] = avg_z;
        table_plane_trans_flat.setOrigin(flat_table_pos);

        // shift the non-flat table frame to the center of the convex hull as well
        table_plane_trans.setOrigin(flat_table_pos);

        // --- [ Take the points projected on the table and transform them into the PointCloud message
        //  while also transforming them into the flat table's coordinate system
        sensor_msgs::PointCloud flat_table_points;
        if (!getPlanePoints<Point>(*table_projected_ptr, table_plane_trans_flat, flat_table_points))
        {
          //TODOresponse.result = response.OTHER_ERROR;
          return ecto::OK;
        }

        // ---[ Create the table message
        // TODO table = getTable<sensor_msgs::PointCloud>(cloud.header, table_plane_trans_flat, flat_table_points);

        // ---[ Convert the convex hull points to flat table frame
        if (!getPlanePoints<Point>(*table_hull_ptr, table_plane_trans_flat, table_hull_points))
        {
          //TODOresponse.result = response.OTHER_ERROR;
          return;
        }
      }

      ROS_INFO("Table computed");
      //TODOresponse.result = response.SUCCESS;

      // ---[ Add the convex hull as a triangle mesh to the Table message
      addConvexHullTable<sensor_msgs::PointCloud>(table, table_hull_points, flatten_table_);

      return ecto::OK;
    }
  private:

    /*! Assumes plane coefficients are of the form ax+by+cz+d=0, normalized */
    tf::Transform
    getPlaneTransform(pcl::ModelCoefficients coeffs, bool flatten_plane)
    {
      double a = coeffs.values[0], b = coeffs.values[1], c = coeffs.values[2], d = coeffs.values[3];
      //asume plane coefficients are normalized
      tf::Vector3 position(-a * d, -b * d, -c * d);
      tf::Vector3 z(a, b, c);

      //if we are flattening the plane, make z just be (0,0,up_direction)
      if (flatten_plane)
      {
        ROS_INFO("flattening plane");
        z[0] = z[1] = 0;
        z[2] = (*up_direction_)[2];
      }
      else
      {
        //make sure z points "up"
        ROS_DEBUG("in getPlaneTransform, z: %0.3f, %0.3f, %0.3f", z[0], z[1], z[2]);
        if (z.dot(*up_direction_) < 0)
        {
          z = -1.0 * z;
          ROS_INFO("flipped z");
        }
      }

      //try to align the x axis with the x axis of the original frame
      //or the y axis if z and x are too close too each other
      tf::Vector3 x(1, 0, 0);
      if (fabs(z.dot(x)) > 1.0 - 1.0e-4)
        x = tf::Vector3(0, 1, 0);
      tf::Vector3 y = z.cross(x).normalized();
      x = y.cross(z).normalized();

      tf::Matrix3x3 rotation;
      rotation[0] = x; // x
      rotation[1] = y; // y
      rotation[2] = z; // z
      rotation = rotation.transpose();
      tf::Quaternion orientation;
      rotation.getRotation(orientation);
      ROS_DEBUG("in getPlaneTransform, x: %0.3f, %0.3f, %0.3f", x[0], x[1], x[2]);
      ROS_DEBUG("in getPlaneTransform, y: %0.3f, %0.3f, %0.3f", y[0], y[1], y[2]);
      ROS_DEBUG("in getPlaneTransform, z: %0.3f, %0.3f, %0.3f", z[0], z[1], z[2]);
      return tf::Transform(orientation, position);
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
        ROS_ERROR("Can not transform point cloud from frame %s to table frame; error %s",
                  table_points.header.frame_id.c_str(), error_msg.c_str());
        return false;
      }
      try
      {
        listener.transformPointCloud("table_frame", table_points, table_points);
      } catch (tf::TransformException ex)
      {
        ROS_ERROR("Failed to transform point cloud from frame %s into table_frame; error %s",
                  table_points.header.frame_id.c_str(), ex.what());
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
      visualization_msgs::Marker tableMarker = MarkerGenerator::getConvexHullTableMarker(table.convex_hull);
      tableMarker.header = table.pose.header;
      tableMarker.pose = table.pose.pose;
      tableMarker.ns = "tabletop_node";
      tableMarker.id = current_marker_id_++;
      marker_pub_.publish(tableMarker);

      visualization_msgs::Marker originMarker = MarkerGenerator::createMarker(table.pose.header.frame_id, 0, .0025,
                                                                              .0025, .01, 0, 1, 1,
                                                                              visualization_msgs::Marker::CUBE,
                                                                              current_marker_id_++, "tabletop_node",
                                                                              table.pose.pose);
      marker_pub_.publish(originMarker);
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

      visualization_msgs::Marker tableMarker = MarkerGenerator::getTableMarker(table.x_min, table.x_max, table.y_min,
                                                                               table.y_max);
      tableMarker.header = cloud_header;
      tableMarker.pose = table_pose;
      tableMarker.ns = "tabletop_node";
      tableMarker.id = current_marker_id_++;
      marker_pub_.publish(tableMarker);

      return table;
    }

    /** The limits of the interest box to find a table, in order [xmin,xmax,ymin,ymax,zmin,zmax] */
    ecto::spore<pcl::ModelCoefficients> table_coefficients_ptr_;
    /** The minimum number of points deemed necessary to find a table */
    ecto::spore<size_t> min_cluster_size_;
    /** The size of a voxel cell when downsampling */
    ecto::spore<float> plane_detection_voxel_size_;
    /** The number of nearest neighbors to use when computing normals */
    ecto::spore<unsigned int> normal_k_search_;
    /** The distance used as a threshold when finding a plane */
    ecto::spore<float> flatten_table_;

    /** flag indicating whether we run in debug mode */
    ecto::spore<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> cloud_, table_projected_ptr_, table_hull_ptr_;
    /** The minimum number of inliers in order to do pose matching */
    ecto::spore<pcl::PointIndices::Ptr> table_inliers_;
    /** The minimum number of inliers in order to do pose matching */
    ecto::spore<pcl::ModelCoefficients::Ptr> table_coefficients_;
    /** The vertical direction */
    ecto::spore<Eigen::Vector3f> up_direction_;
  };

}

ECTO_CELL(tabletop_table, tabletop::TablePublisher, "TablePublisher", "Given a point cloud, find  a potential table.");
