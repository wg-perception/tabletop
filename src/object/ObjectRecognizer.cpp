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

#include <fstream>
#include <iostream>

#include <boost/foreach.hpp>
#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/shared_ptr.hpp>

#include <ecto/ecto.hpp>
#include <Eigen/Geometry>
#ifdef PCL_VERSION_GE_140
#include <pcl/common/transforms.h>
#else
#include <pcl/registration/transforms.h>
#endif

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <household_objects_database/objects_database.h>
#include <tabletop/Table.h>
#include <tabletop/table/tabletop_segmenter.h>
#include <tabletop/object/tabletop_object_detector.h>

#include <object_recognition_core/common/pose_result.h>
#include <object_recognition_core/common/types.h>

using object_recognition_core::common::PoseResult;

using ecto::tendrils;

namespace tabletop
{
  /** Ecto implementation of a module that recognizes objects using the tabletop code
   *
   */
  struct ObjectRecognizer
  {
    void
    ParameterCallback(const std::string &model_set)
    {
      //std::vector<object_recognition_core::db::ModelId> object_ids;

      //boost::python::stl_input_iterator<std::string> begin(python_object_ids), end;
      //std::copy(begin, end, std::back_inserter(object_ids));

      object_recognizer_ = tabletop_object_detector::TabletopObjectRecognizer();
      std::stringstream port;
      port << db_->parameters().raw_.find("port")->second.get_int();

      household_objects_database::ObjectsDatabase *database = new household_objects_database::ObjectsDatabase(
          db_->parameters().raw_.find("host")->second.get_str(), port.str(),
          db_->parameters().raw_.find("user")->second.get_str(),
          db_->parameters().raw_.find("password")->second.get_str(),
          db_->parameters().raw_.find("name")->second.get_str());

      std::vector<boost::shared_ptr<household_objects_database::DatabaseScaledModel> > models;
      if (!database->getScaledModelsBySet(models, model_set))
        return;

      object_recognizer_.clearObjects();
      for (size_t i = 0; i < models.size(); i++)
      {
        int model_id = models[i]->id_.data();
        arm_navigation_msgs::Shape mesh;

        if (!database->getScaledModelMesh(model_id, mesh))
          continue;

        object_recognizer_.addObject(model_id, mesh);
      }
    }

    static void
    declare_params(ecto::tendrils& params)
    {
      params.declare(&ObjectRecognizer::object_ids_, "object_ids",
                     "The DB id of the objects to load in the household database.").required(true);
      params.declare(&ObjectRecognizer::db_, "db", "The DB parameters").required(true);
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&ObjectRecognizer::clusters_, "clusters", "The object clusters.").required(true);
      inputs.declare(&ObjectRecognizer::table_rotations_, "rotations", "The pose rotations of the tables.").required(
          true);
      inputs.declare(&ObjectRecognizer::table_translations_, "translations", "The pose translations of the tables").required(
          true);

      outputs.declare(&ObjectRecognizer::pose_results_, "pose_results", "The results of object recognition");
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      object_ids_.set_callback(boost::bind(&ObjectRecognizer::ParameterCallback, this, _1));
      object_ids_.dirty(true);

      perform_fit_merge_ = true;
      confidence_cutoff_ = 0.01;
    }

    /** Compute the pose of the table plane
     * @param inputs
     * @param outputs
     * @return
     */
    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      std::vector<tabletop_object_detector::TabletopObjectRecognizer::TabletopResult<pcl::PointXYZ> > results;

      // Process each table
      pose_results_->clear();
      for (size_t table_index = 0; table_index < clusters_->size(); ++table_index)
      {
        Eigen::Quaternionf quat((*table_rotations_)[table_index]);

        // Make the clusters be in the table frame
        size_t n_clusters = (*clusters_)[table_index].size();
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters(n_clusters);

        for (size_t cluster_index = 0; cluster_index < n_clusters; ++cluster_index)
        {
          Eigen::Matrix4f t;
          t.block(0, 0, 3, 3) = (*table_rotations_)[table_index].transpose();
          t.block(0, 3, 3, 1) = -(*table_rotations_)[table_index].transpose() * (*table_translations_)[table_index];

          clusters[cluster_index] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
          pcl::transformPointCloud(*((*clusters_)[table_index][cluster_index]), *(clusters[cluster_index]), t);
        }
        object_recognizer_.objectDetection<pcl::PointXYZ>(clusters, confidence_cutoff_, perform_fit_merge_, results);

        for (size_t i = 0; i < results.size(); ++i)
        {
          const tabletop_object_detector::TabletopObjectRecognizer::TabletopResult<pcl::PointXYZ> & result = results[i];

          PoseResult pose_result;
          const geometry_msgs::Pose &pose = result.pose_;
          std::stringstream ss;
          ss << result.object_id_;
          pose_result.set_object_id(*db_, ss.str());

          Eigen::Vector3f T(pose.position.x, pose.position.y, pose.position.z);
          Eigen::Quaternionf quat(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

          Eigen::Vector3f new_T = (*table_rotations_)[table_index] * T + (*table_translations_)[table_index];
          pose_result.set_T(new_T);

          pose_result.set_R(quat);
          Eigen::Matrix3f R = (*table_rotations_)[table_index] * pose_result.R<Eigen::Matrix3f>();
          pose_result.set_R(R);
          pose_result.set_confidence(result.confidence_);

          pose_results_->push_back(pose_result);
        }
      }

      return ecto::OK;
    }
  private:
    typedef std::vector<tabletop_object_detector::ModelFitInfo> ModelFitInfos;
    /** The object recognizer */
    tabletop_object_detector::TabletopObjectRecognizer object_recognizer_;
    /** The resulting poses of the objects */
    ecto::spore<std::vector<PoseResult> > pose_results_;
    ecto::spore<std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> > > clusters_;
    /** The rotations of the tables */
    ecto::spore<std::vector<Eigen::Matrix3f> > table_rotations_;
    /** The translations of the tables */
    ecto::spore<std::vector<Eigen::Vector3f> > table_translations_;
    /** The number of models to fit to each cluster */
    int confidence_cutoff_;
    bool perform_fit_merge_;
    ecto::spore<std::string> object_ids_;
    ecto::spore<object_recognition_core::db::ObjectDb> db_;
  };
}

ECTO_CELL(tabletop_object, tabletop::ObjectRecognizer, "ObjectRecognizer",
          "Given clusters on a table, identify them as objects.");
