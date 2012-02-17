
/*********************************************************************
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/


void TabletopObjectRecognizer::publishFitMarkers(
                 const std::vector<household_objects_database_msgs::DatabaseModelPoseList> &potential_models,
                 const Table &table)
{
  for (size_t i=0; i<potential_models.size(); i++)
  {
    const std::vector<household_objects_database_msgs::DatabaseModelPose> models = potential_models[i].model_list;
    for (size_t j=0; j<models.size(); j++)
    {
      if (models[j].confidence > min_marker_quality_) break;
      household_objects_database_msgs::GetModelMesh get_mesh;
      get_mesh.request.model_id = models[j].model_id;
      if ( !get_model_mesh_srv_.call(get_mesh) ||
           get_mesh.response.return_code.code != get_mesh.response.return_code.SUCCESS )
      {
        ROS_ERROR("tabletop_object_detector: failed to call database get mesh service for marker display");
      }
      else
      {
        double rank = ((double)j) / std::max( (int)(models.size())-1, 1 );
        visualization_msgs::Marker fitMarker =  MarkerGenerator::getFitMarker(get_mesh.response.mesh, rank);
        fitMarker.header = table.pose.header;
        fitMarker.pose = models[j].pose.pose;
        fitMarker.ns = "tabletop_node_model_" + boost::lexical_cast<std::string>(j);
        fitMarker.id = current_marker_id_++;
        marker_pub_.publish(fitMarker);
      }
    }
  }
}

void TabletopObjectRecognizer::clearOldMarkers(std::string frame_id)
{
  for (int id=current_marker_id_; id < num_markers_published_; id++)
  {
    visualization_msgs::Marker delete_marker;
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.header.frame_id = frame_id;
    delete_marker.id = id;
    delete_marker.action = visualization_msgs::Marker::DELETE;
    //a hack, but we don't know what namespace the marker was in
    for (size_t j=0; j<10; j++)
    {
      delete_marker.ns = "tabletop_node_model_" + boost::lexical_cast<std::string>(j);
      marker_pub_.publish(delete_marker);
    }
  }
  num_markers_published_ = current_marker_id_;
  current_marker_id_ = 0;
}
