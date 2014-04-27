/*********************************************************************
 * Software License Agreement (BSD License)
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

// Author(s): Marius Muja and Matei Ciocarlie

#ifndef _EXHAUSTIVE_FIT_DETECTOR_
#define _EXHAUSTIVE_FIT_DETECTOR_

#include <vector>
#include <set>

#include <opencv2/flann/flann.hpp>

#include <shape_msgs/Mesh.h>

#include <tabletop_object_detector/model_fitter.h>

namespace tabletop_object_detector {

//! Given a point cloud, computes the fit against multiple meshes and chooses the best ones
/*! Is templated on the type of individual fitter to use; the individual fitter must
  be able to store a mesh or point cloud inside of it, then later compute the fit to
  an input cloud. The individual fitter is designed to inherit from ModelToCloudFitter,
  defined inside of model_fitter.h
  
  This class just initializes a whole bunch of individual fitters, then, when given a 
  new cloud, tries to fit all of them. It will then return the fits with the best scores.  
*/
template <class Fitter>
class ExhaustiveFitDetector
{
 private:
  //! Stores the individual model to cloud fitters, each initialized with a model
  std::vector<Fitter*> templates;

  //! Stores a list of model ids which may be in the list of templates, but which we should not look at
  std::set<int> model_exclusion_set_;
  bool negate_exclusions_;

 public:
  //! Just a stub; does not load models
 ExhaustiveFitDetector() : negate_exclusions_(false) {}
  //! Deletes any loaded models
  ~ExhaustiveFitDetector();

  void addModelToExclusionList(int model_id)
  {
    model_exclusion_set_.insert(model_id);
  }

  void clearExclusionList()
  {
    model_exclusion_set_.clear();
  }

  void setNegateExclusions(bool value)
  {
    negate_exclusions_ = value;
  }

  //! Loads all the models that are in the model database
    void
    loadDatabaseModels(const std::string &model_set, const std::string &database_host, const std::string database_port,
                       const std::string &database_user, const std::string &database_pass,
                       const std::string &database_name);

    void
    clearObjects()
    {
      templates.clear();
    }

    void
    addObject(int model_id, const shape_msgs::Mesh & mesh)
    {
      Fitter* fitter = new Fitter();
      fitter->initializeFromMesh(mesh);
      templates.push_back(fitter);
      //set the model ID in the template so that we can use it later
      templates.back()->setModelId(model_id);
    }

  //! Main fitting function; fits all meshes against \a cloud and sorts the fits
  /*! Fits the point cloud \a cloud against all the models in the internal list.
    It always stores the list with at most \a numResults best fits, sorted by 
    their score. At the end, it returns this list.
  */
  std::vector<ModelFitInfo> fitBestModels(const std::vector<cv::Vec3f>& cloud, int numResults, cv::flann::Index &search, double min_object_score)
  {
    std::vector<ModelFitInfo> fit_results;
    if (numResults <= 0) return fit_results;
    
    for (size_t i=0; i<templates.size(); ++i) 
    {
      ModelFitInfo current = templates[i]->fitPointCloud(cloud, search, min_object_score);
      // If the model that was matched is not in the exclusion list
      bool found = (model_exclusion_set_.find(current.getModelId()) != model_exclusion_set_.end());
      if (negate_exclusions_ == found)
      {
        if ((int)fit_results.size() < numResults)
        {
          fit_results.push_back(current);
          std::sort(fit_results.begin(), fit_results.end(), ModelFitInfo::compareScores);
        }
        else
        {
          if (fit_results.back().getScore() < current.getScore())
          {
            fit_results.back() = current;
            std::sort(fit_results.begin(), fit_results.end(), ModelFitInfo::compareScores);
          }
        }
      }
    } 
    return fit_results;
  }
};

template <class Fitter>
ExhaustiveFitDetector<Fitter>::~ExhaustiveFitDetector()
{
  for (size_t i=0;i<templates.size();++i) {
    delete templates[i];
  }
}

} //namespace

#endif
