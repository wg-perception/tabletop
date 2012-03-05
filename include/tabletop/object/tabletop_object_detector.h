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

#include <pcl/point_cloud.h>

#include "tabletop_object_detector/exhaustive_fit_detector.h"
#include "tabletop_object_detector/marker_generator.h"
#include "tabletop_object_detector/iterative_distance_fitter.h"
#include "tabletop/Table.h"

namespace tabletop_object_detector
{
  class TabletopObjectRecognizer
  {
  private:
    //! The instance of the detector used for all detecting tasks
    ExhaustiveFitDetector<IterativeTranslationFitter> detector_;

    //! The threshold for merging two models that were fit very close to each other
    double fit_merge_threshold_;

  public:
    //! Subscribes to and advertises topics; initializes fitter
    TabletopObjectRecognizer();

    //! Empty stub
    ~TabletopObjectRecognizer()
    {
    }

    void
    clearObjects();

    void
    addObject(int model_id, arm_navigation_msgs::Shape mesh);

    /** Structure used a return type for objectDetection */
    template<class PointType>
    struct TabletopResult
    {
      geometry_msgs::Pose pose_;
      float confidence_;
      int object_id_;
      typename pcl::PointCloud<PointType>::Ptr cloud_;
    };

    /*! Performs the detection on each of the clusters, and populates the returned message.
     */
    template<class PointType>
    void
    objectDetection(std::vector<typename pcl::PointCloud<PointType>::Ptr> &clusters, float confidence_cutoff,
                    bool perform_fit_merge, std::vector<TabletopResult<PointType> > &results)
    {
      //do the model fitting part
      std::vector<size_t> cluster_model_indices;
      std::vector<std::vector<ModelFitInfo> > raw_fit_results(clusters.size());
      cluster_model_indices.resize(clusters.size(), -1);
      int num_models = 10;
      for (size_t i = 0; i < clusters.size(); i++)
      {
        std::vector<ModelFitInfo> fit_results = detector_.fitBestModels<typename pcl::PointCloud<PointType> >(
            *(clusters[i]), std::max(1, num_models));
        std::vector<ModelFitInfo> &final_fit_results = raw_fit_results[i];

        final_fit_results.reserve(fit_results.size());
        BOOST_FOREACH(const ModelFitInfo & fit_info, fit_results)
            {
              if (fit_info.getScore() >= confidence_cutoff)
                final_fit_results.push_back(fit_info);
            }

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
              if (fitClusterDistance<typename pcl::PointCloud<PointType> >(raw_fit_results.at(i).at(0), *clusters[j])
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
            clusters[i]->points.insert(clusters[i]->points.end(), clusters[j]->points.begin(),
                                       clusters[j]->points.end());
            //delete fits for cluster j so we ignore it from now on
            raw_fit_results.at(j).clear();
            //fits for cluster j now point at fit for cluster i
            cluster_model_indices[j] = i;
            //refit cluster i
            raw_fit_results.at(i) = detector_.fitBestModels(*(clusters[i]), std::max(1, num_models));
          }
          else
          {
            i++;
          }
        }
      }

      // Merge clusters together
      for (size_t i = 0; i < cluster_model_indices.size(); i++)
      {
        if ((cluster_model_indices[i] != int(i)) || (raw_fit_results[i].empty()))
          continue;

        TabletopResult<PointType> result;
        result.object_id_ = raw_fit_results[i][0].getModelId();
        result.pose_ = raw_fit_results[i][0].getPose();
        result.confidence_ = raw_fit_results[i][0].getScore();
        result.cloud_ = typename pcl::PointCloud<PointType>::Ptr(new typename pcl::PointCloud<PointType>());

        for (size_t j = i; j < cluster_model_indices.size(); j++)
        {
          if (raw_fit_results[j][0].getScore() > result.confidence_)
          {
            result.pose_ = raw_fit_results[j][0].getPose();
            result.confidence_ = raw_fit_results[j][0].getScore();
          }
          // Merge the points in the same point cloud
          *(result.cloud_) += (*clusters[j]);
        }

        results.push_back(result);
      }
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
  };
}

#endif /* TABLETOP_OBJECT_DETECTOR_H_ */
