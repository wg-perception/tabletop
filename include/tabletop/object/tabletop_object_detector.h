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
//#include <pcl/search/kdtree.h>
//#include <pcl/search/brute_force.h>
//#include <pcl/search/octree.h>
#include <pcl/search/impl/brute_force.hpp>

#include "tabletop_object_detector/exhaustive_fit_detector.h"
#include "tabletop_object_detector/marker_generator.h"
#include "tabletop_object_detector/iterative_distance_fitter.h"

namespace tabletop_object_detector
{
  template <typename PointType>
  class TabletopObjectRecognizer
  {
  private:
    //! The instance of the detector used for all detecting tasks
    ExhaustiveFitDetector<IterativeTranslationFitter<pcl::PointCloud<PointType> > > detector_;

    //! The threshold for merging two models that were fit very close to each other
    double fit_merge_threshold_;

    double getConfidence (double score) const
    {
      return (1.0 - (1.0 - score) * (1.0 - score));
    }
  public:
    //! Subscribes to and advertises topics; initializes fitter
    TabletopObjectRecognizer()
    {
      detector_ = ExhaustiveFitDetector<IterativeTranslationFitter<pcl::PointCloud<PointType> > >();
      //initialize operational flags
      fit_merge_threshold_ = 0.02;
    }

    //! Empty stub
    ~TabletopObjectRecognizer()
    {
    }

    void
    clearObjects()
    {
      detector_.clearObjects();
    }

    void
    addObject(int model_id, const shape_msgs::Mesh & mesh)
    {
      detector_.addObject(model_id, mesh);
    }

    /** Structure used a return type for objectDetection */
    struct TabletopResult
    {
      geometry_msgs::Pose pose_;
      float confidence_;
      int object_id_;
      typename pcl::PointCloud<PointType>::Ptr cloud_;
    };

    /*! Performs the detection on each of the clusters, and populates the returned message.
     */
    void
    objectDetection(std::vector<typename pcl::PointCloud<PointType>::Ptr> &clusters, float confidence_cutoff,
                    bool perform_fit_merge, std::vector<TabletopResult > &results)
    {
      //do the model fitting part
      std::vector<size_t> cluster_model_indices;
      std::vector<std::vector<ModelFitInfo> > raw_fit_results(clusters.size());
      std::vector<typename pcl::search::Search<PointType>::Ptr> search (clusters.size ());
      cluster_model_indices.resize(clusters.size(), -1);
      int num_models = 1;
      for (size_t i = 0; i < clusters.size(); i++)
      {
        cluster_model_indices[i] = i;
	search[i].reset (new pcl::search::BruteForce<PointType> ());
	//        search[i]->setInputCloud (clusters[i]);
        std::vector<ModelFitInfo> fit_results = detector_.fitBestModels (
            *(clusters[i]), std::max(1, num_models), *search[i]);
        std::vector<ModelFitInfo> &final_fit_results = raw_fit_results[i];
        final_fit_results.reserve(fit_results.size());
        BOOST_FOREACH(const ModelFitInfo & fit_info, fit_results)
        {
          if (getConfidence(fit_info.getScore()) >= confidence_cutoff)
            final_fit_results.push_back(fit_info);
        }
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
//            if (raw_fit_results.at(j).empty())
//            {
//              if (fitClusterDistance<typename pcl::PointCloud<PointType> >(raw_fit_results.at(i).at(0), *clusters[j])
//                  < fit_merge_threshold_)
//                break;
//              else
//                continue;
//            }
            //else merge based on fits
            if (!raw_fit_results.at(j).empty() && fitDistance(raw_fit_results.at(i).at(0), raw_fit_results.at(j).at(0)) < fit_merge_threshold_)
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
            raw_fit_results.at(i) = detector_.fitBestModels(*(clusters[i]), std::max(1, num_models), *search[i]);
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

        double confidence = getConfidence (raw_fit_results[i][0].getScore());

        if (confidence < confidence_cutoff)
          continue;

        TabletopResult result;
        result.object_id_ = raw_fit_results[i][0].getModelId();
        result.pose_ = raw_fit_results[i][0].getPose();
        result.confidence_ = confidence;
        result.cloud_ = clusters[i];
        result.cloud_->width = result.cloud_->size();

        results.push_back(result);
      }
    }

    //-------------------- Misc -------------------

    //! Helper function that returns the distance along the plane between two fit models
    double
    fitDistance(const ModelFitInfo &m1, const ModelFitInfo &m2)
    {
      double dx = m1.getPose().position.x - m2.getPose().position.x;
      double dy = m1.getPose().position.y - m2.getPose().position.y;
      double d = dx * dx + dy * dy;
      return sqrt(d);
    }

    template<class PointCloudType>
    double
    fitClusterDistance(const ModelFitInfo &m, const PointCloudType &cluster)
    {
      double dist = 100.0 * 100.0;
      double mx = m.getPose().position.x;
      double my = m.getPose().position.y;
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
