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
/** File defining a cell producing messages for tabletop visualization
 */

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <ecto/ecto.hpp>

#include <sensor_msgs/Image.h>

#include <object_recognition_msgs/Table.h>
#include <object_recognition_msgs/TableArray.h>

//for random colors
#include <stdlib.h>
#include <time.h>

#include <vector>

#include <opencv2/core/core.hpp>

#include <visualization_msgs/Marker.h>

using ecto::tendrils;


/*!
 *  It is the responsibility of the caller to set the appropriate pose for the marker so that
 *  it shows up in the right reference frame.
 */
visualization_msgs::Marker getCloudMarker(const std::vector<cv::Vec3f>& cloud)
{
  static bool first_time = true;
  if (first_time) {
    srand ( time(NULL) );
    first_time = false;
  }

  //create the marker
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(5);

  marker.type = visualization_msgs::Marker::POINTS;
  marker.scale.x = 0.002;
  marker.scale.y = 0.002;
  marker.scale.z = 1.0;

  marker.color.r = ((double)rand())/RAND_MAX;
  marker.color.g = ((double)rand())/RAND_MAX;
  marker.color.b = ((double)rand())/RAND_MAX;
  marker.color.a = 1.0;

  for(size_t i=0; i<cloud.size(); i++) {
    geometry_msgs::Point p;
    p.x = cloud[i][0];
    p.y = cloud[i][1];
    p.z = cloud[i][2];
    marker.points.push_back(p);
  }

  //the caller must decide the header; we are done here
  return marker;
}

/** Class simplifying the management of MarkerArray in RViz
 */
class MarkerArrayWrapper
{
public:
  MarkerArrayWrapper() :
    base_id_(0) {
  }

  /**
   * @param base_id The id of the markers will go from base_id_ to base_id_ + 100 at most
   */
  MarkerArrayWrapper(int base_id) :
    base_id_(base_id) {
  }

  /** This deletes all the Markers that were marked to be deleted and sets the ones that
   * were added to now be deleted
   */
  void
  clear() {
    // Delete all the last markers that were already set for deletion
    size_t size = array_.markers.size();
    for (visualization_msgs::MarkerArray::_markers_type::reverse_iterator iter =
           array_.markers.rbegin(), iter_end = array_.markers.rend(); iter != iter_end; ++iter) {
      if (iter->action == visualization_msgs::Marker::DELETE)
        --size;
      else
        break;
    }
    array_.markers.resize(size);

    // Set the other markers for deletion by default
    for (visualization_msgs::MarkerArray::_markers_type::iterator iter =
           array_.markers.begin(), iter_end = array_.markers.end(); iter != iter_end; ++iter) {
      iter->action = visualization_msgs::Marker::DELETE;
    }
  }

  void
  push_back(const visualization_msgs::Marker& marker) {
    // Find the first element that is not assigned
    visualization_msgs::MarkerArray::_markers_type::iterator iter =
      array_.markers.begin(), iter_end = array_.markers.end();
    for (; iter != iter_end; ++iter) {
      if (iter->action == visualization_msgs::Marker::DELETE)
        break;
    }
    // Add the marker and modify its id
    visualization_msgs::Marker* last;
    if (iter == iter_end) {
      array_.markers.push_back(marker);
      last = &array_.markers.back();
    } else {
      *iter = marker;
      last = &(*iter);
    }
    last->id = base_id_ + (iter - array_.markers.begin());
    last->action = visualization_msgs::Marker::ADD;
    last->lifetime = ros::Duration(5, 0);
  }

  const visualization_msgs::MarkerArray
  array() const {
    return array_;
  }
private:
  /** The id of the markers will go from base_id_ to base_id_ + 100 at most */
  int base_id_;
  /** The container of all the markers */
  visualization_msgs::MarkerArray array_;
};

/** Ecto cells that creates messages containing what has been found by the tabletop pipeline
 */
struct TableVisualizationMsgAssembler {
  static void declare_io(const tendrils& params, tendrils& inputs,
                         tendrils& outputs) {
    inputs.declare(&TableVisualizationMsgAssembler::clusters3d_, "clusters3d",
                   "The clusters on top of the table.").required(true);
    inputs.declare(&TableVisualizationMsgAssembler::image_message_,
                   "image_message", "the image message to get the header").required(true);

    outputs.declare<visualization_msgs::MarkerArrayConstPtr>(
      "marker_array_clusters", "The markers of the clusters");
  }

  void configure(const tendrils& params, const tendrils& inputs,
                 const tendrils& outputs) {
    marker_array_clusters_ = MarkerArrayWrapper(0);
  }

  int process(const tendrils& inputs, const tendrils& outputs) {
    std_msgs::Header message_header;
    if (*image_message_)
      message_header = (*image_message_)->header;
    // Delete the old markers
    marker_array_clusters_.clear();

    // Publish each clusters
    for (size_t i = 0; i < clusters3d_->size(); ++i) {
      std::vector<std::vector<cv::Vec3f> > &clusters = (*clusters3d_)[i];
      for (size_t j = 0; j < clusters.size(); j++) {
        visualization_msgs::Marker cloud_marker =
        getCloudMarker(clusters[j]);
        cloud_marker.header = message_header;
        cloud_marker.pose.orientation.w = 1;
        cloud_marker.ns = "tabletop_node";
        marker_array_clusters_.push_back(cloud_marker);
      }
    }

    outputs["marker_array_clusters"]
        << visualization_msgs::MarkerArrayConstPtr(
          new visualization_msgs::MarkerArray(marker_array_clusters_.array()));

    return ecto::OK;
  }
private:

  /** The image message the initial data is from */
  ecto::spore<sensor_msgs::ImageConstPtr> image_message_;

  MarkerArrayWrapper marker_array_clusters_;
  /** For each table, a vector of clusters */
  ecto::spore<std::vector<std::vector<std::vector<cv::Vec3f> > > > clusters3d_;
};

ECTO_CELL(tabletop_table, TableVisualizationMsgAssembler,
          "TableVisualizationMsgAssembler",
          "Given a point cloud, find  a potential table.");
