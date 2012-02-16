/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

#ifndef TABLETOP_OBJECT_DETECTOR_H_
#define TABLETOP_OBJECT_DETECTOR_H_

// Author(s): Marius Muja and Matei Ciocarlie

#include <string>

#include "tabletop_object_detector/exhaustive_fit_detector.h"
#include "tabletop_object_detector/marker_generator.h"
#include "tabletop_object_detector/iterative_distance_fitter.h"
#include "tabletop/Table.h"

namespace tabletop_object_detector
{
  class TabletopObjectRecognizer
  {
  private:
    //! Fit results below this quality are not published as markers
    double min_marker_quality_;
    //! Used to remember the number of markers we publish so we can delete them later
    int num_markers_published_;
    //! The current marker being published
    int current_marker_id_;

    //! The instance of the detector used for all detecting tasks
    ExhaustiveFitDetector<IterativeTranslationFitter> detector_;

    //! Whether to use a reduced model set from the database
    std::string model_set_;

    //! The threshold for merging two models that were fit very close to each other
    double fit_merge_threshold_;

    /*! Performs the detection on each of the clusters, and populates the returned message.
     */
    template<class PointCloudType>
    void
    objectDetection(std::vector<PointCloudType> &clusters, int num_models, const tabletop::Table &table,
                    bool perform_fit_merge, std::vector<std::vector<ModelFitInfo> > &raw_fit_results,
                    std::vector<size_t> &cluster_model_indices)
    {
      //do the model fitting part
      cluster_model_indices.resize(clusters.size(), -1);
      for (size_t i = 0; i < clusters.size(); i++)
      {
        raw_fit_results.push_back(detector_.fitBestModels<PointCloudType>(clusters[i], std::max(1, num_models)));
        cluster_model_indices[i] = i;
      }

      //merge models that were fit very close to each other
      if (perform_fit_merge)
      {
        size_t i = 0;
        while (i < clusters.size())
        {
          //if cluster i has already been merged continue
          if (cluster_model_indices[i] != (int) i || raw_fit_results.at(i).empty())
          {
            i++;
            continue;
          }

          size_t j;
          for (j = i + 1; j < clusters.size(); j++)
          {
            //if cluster j has already been merged continue
            if (cluster_model_indices[j] != (int) j)
              continue;
            //if there are no fits, merge based on cluster vs. fit
            if (raw_fit_results.at(j).empty())
            {
              if (fitClusterDistance<PointCloudType>(raw_fit_results.at(i).at(0), clusters.at(j))
                  < fit_merge_threshold_)
                break;
              else
                continue;
            }
            //else merge based on fits
            if (fitDistance(raw_fit_results.at(i).at(0), raw_fit_results.at(j).at(0)) < fit_merge_threshold_)
              break;
          }
          if (j < clusters.size())
          {
            //merge cluster j into i
            clusters[i].points.insert(clusters[i].points.end(), clusters[j].points.begin(), clusters[j].points.end());
            //delete fits for cluster j so we ignore it from now on
            raw_fit_results.at(j).clear();
            //fits for cluster j now point at fit for cluster i
            cluster_model_indices[j] = i;
            //refit cluster i
            raw_fit_results.at(i) = detector_.fitBestModels(clusters[i], std::max(1, num_models));
          }
          else
          {
            i++;
          }
        }
      }

      //make sure raw clusters point at the right index in fit_models
      for (size_t i = 0; i < raw_fit_results.size(); i++)
      {
        if (cluster_model_indices[i] != (int) i)
        {
          int ind = cluster_model_indices[i];
          cluster_model_indices[i] = cluster_model_indices[ind];
          //ROS_INFO("  - has been merged with fit for cluster %d", ind);
        }
      }

      /*tf::Transform table_trans;
       tf::poseMsgToTF(table.pose.pose, table_trans);
       for (size_t i = 0; i < raw_fit_results.size(); i++)
       {
       household_objects_database_msgs::DatabaseModelPoseList model_potential_fit_list;
       //prepare the actual result for good fits, only these are returned
       for (size_t j = 0; j < raw_fit_results[i].size(); j++)
       {
       //get the model pose in the cloud frame by multiplying with table transform
       tf::Transform model_trans;
       tf::poseMsgToTF(raw_fit_results[i][j].getPose(), model_trans);
       model_trans = table_trans * model_trans;
       geometry_msgs::Pose model_pose;
       tf::poseTFToMsg(model_trans, model_pose);
       //create the model fit result
       household_objects_database_msgs::DatabaseModelPose pose_msg;
       pose_msg.model_id = raw_fit_results[i][j].getModelId();
       pose_msg.pose.header = table.pose.header;
       pose_msg.pose.pose = model_pose;
       pose_msg.confidence = raw_fit_results[i][j].getScore();
       //and push it in the list for this cluster
       model_potential_fit_list.model_list.push_back(pose_msg);
       }
       response.models.push_back(model_potential_fit_list);
       }*/
    }

    //-------------------- Misc -------------------

    //! Helper function that returns the distance along the plane between two fit models
    double
    fitDistance(const ModelFitInfo &m1, const ModelFitInfo &m2);

    template<class PointCloudType>
    double
    fitClusterDistance(const ModelFitInfo &m, const PointCloudType &cluster)
    {
      double dist = 100.0 * 100.0;
      double mx = m.getPose().position.x;
      double my = m.getPose().position.x;
      for (size_t i = 0; i < cluster.points.size(); i++)
      {
        double dx = cluster.points[i].x - mx;
        double dy = cluster.points[i].y - my;
        double d = dx * dx + dy * dy;
        dist = std::min(d, dist);
      }
      return sqrt(dist);
    }

  public:
    //! Subscribes to and advertises topics; initializes fitter
    TabletopObjectRecognizer();

    //! Empty stub
    ~TabletopObjectRecognizer()
    {
    }
  };
}

#endif /* TABLETOP_OBJECT_DETECTOR_H_ */
