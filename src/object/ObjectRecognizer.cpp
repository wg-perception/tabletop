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
#include <sstream>

#include <boost/foreach.hpp>
#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/shared_ptr.hpp>

#include <ecto/ecto.hpp>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include "point_cloud2_proxy.h"

#include <household_objects_database/objects_database.h>
#include <tabletop/object/tabletop_object_detector.h>

#include <opencv2/core/core.hpp>

#include <object_recognition_core/common/pose_result.h>
#include <object_recognition_core/common/types.h>
#include <object_recognition_core/db/ModelReader.h>

#include <object_recognition_tabletop/household.h>

#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

using object_recognition_core::common::PoseResult;

using ecto::tendrils;

/**
 * If the equation of the plane is ax+by+cz+d=0, the pose (R,t) is such that it takes the horizontal plane (z=0)
 * to the current equation
 */
void
getPlaneTransform(const cv::Vec4f& plane_coefficients, cv::Matx33f& rotation, cv::Vec3f& translation)
{
  double a = plane_coefficients[0], b = plane_coefficients[1], c = plane_coefficients[2], d = plane_coefficients[3];
  // assume plane coefficients are normalized
  translation = cv::Vec3f(-a * d, -b * d, -c * d);
  cv::Vec3f z(a, b, c);

  //try to align the x axis with the x axis of the original frame
  //or the y axis if z and x are too close too each other
  cv::Vec3f x(1, 0, 0);
  if (fabs(z.dot(x)) > 1.0 - 1.0e-4)
    x = cv::Vec3f(0, 1, 0);
  cv::Vec3f y = z.cross(x);
  x = y.cross(z);
  x = x / norm(x);
  y = y / norm(y);

  rotation = cv::Matx33f(x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]);
}

namespace tabletop
{
/** Ecto implementation of a module that recognizes objects using the tabletop code
 */
struct ObjectRecognizer : public object_recognition_core::db::bases::ModelReaderBase {
  virtual
  void parameter_callback(const object_recognition_core::db::Documents& db_documents) {
    object_recognizer_.clearObjects();

    aiLogStream* ai_stream_ = new aiLogStream(aiGetPredefinedLogStream(aiDefaultLogStream_STDOUT, NULL));
    aiAttachLogStream(ai_stream_);

    int template_db_id = 0;
    BOOST_FOREACH(const object_recognition_core::db::Document & document, db_documents) {
      // Get the list of _attachments and figure out the original mesh
      std::vector<std::string> attachments_names = document.attachment_names();
      std::string mesh_path;
      std::vector<std::string> possible_names(2);
      possible_names[0] = "original";
      possible_names[1] = "mesh";
      for (size_t i = 0; i < 2 && mesh_path.empty(); ++i) {
        BOOST_FOREACH(const std::string & attachment_name, attachments_names) {
          if (attachment_name.find(possible_names[i]) != 0)
            continue;
          // Create a temporary file
          char mesh_path_tmp[L_tmpnam];
          tmpnam(mesh_path_tmp);
          mesh_path = std::string(mesh_path_tmp) + attachment_name.substr(possible_names[i].size());

          // Load the mesh and save it to the temporary file
          std::ofstream mesh_file;
          mesh_file.open(mesh_path.c_str());
          document.get_attachment_stream(attachment_name, mesh_file);
          mesh_file.close();
          break;
        }
      }

      household_id_to_db_id_[template_db_id] = document.get_field<std::string>("object_id");

      // Load the mesh through assimp
      std::cout << "Loading model: " << document.id() << " for object id: " << household_id_to_db_id_[template_db_id];

      const struct aiScene* scene = aiImportFile(mesh_path.c_str(), aiProcess_FindDegenerates |
      aiProcess_FindInvalidData |
      aiProcess_ImproveCacheLocality |
      aiProcess_JoinIdenticalVertices |
      aiProcess_OptimizeGraph |
      aiProcess_OptimizeMeshes |
      aiProcess_RemoveRedundantMaterials |
      aiProcess_SortByPType |
      aiProcess_Triangulate |
      aiProcess_RemoveComponent |
      aiProcess_FlipUVs |
      aiProcess_ValidateDataStructure |
      aiProcess_MakeLeftHanded);
      const aiNode* nd = scene->mRootNode;

      // Load the meshes and convert them to a shape_msgs::Mesh
      shape_msgs::Mesh mesh_msg;
      double min_z = std::numeric_limits<double>::max();
      for (size_t i_mesh = 0; i_mesh < scene->mNumMeshes; ++i_mesh) {
        const struct aiMesh* mesh = scene->mMeshes[i_mesh];
        size_t size_ini = mesh_msg.vertices.size();
        mesh_msg.vertices.resize(size_ini + mesh->mNumVertices);
        for (size_t j = 0; j < mesh->mNumVertices; ++j) {
          const aiVector3D& vertex = mesh->mVertices[j];
          mesh_msg.vertices[size_ini + j].x = vertex.x;
          mesh_msg.vertices[size_ini + j].y = vertex.y;
          mesh_msg.vertices[size_ini + j].z = vertex.z;
          if (vertex.z < min_z)
            min_z = vertex.z;
        }

        size_t size_ini_triangles = mesh_msg.triangles.size();
        mesh_msg.triangles.resize(size_ini_triangles + mesh->mNumFaces);
        size_t j_triangles = size_ini_triangles;
        for (size_t j = 0; j < mesh->mNumFaces; ++j) {
          const aiFace& face = mesh->mFaces[j];
          if (face.mNumIndices == 3) {
            for (size_t k = 0; k < 3; ++k)
              mesh_msg.triangles[j_triangles].vertex_indices[k] = size_ini + face.mIndices[k];
            ++j_triangles;
          }
        }
        mesh_msg.triangles.resize(j_triangles);
      }

      // Make sure z=0 is the minimum z for this mesh
      for (size_t i = 0; i < mesh_msg.vertices.size(); ++i)
        mesh_msg.vertices[i].z -= min_z;

      min_z_[document.get_field<std::string>("object_id")] = min_z;

      object_recognizer_.addObject(template_db_id, mesh_msg);

      std::cout << std::endl;

      aiReleaseImport(scene);
      template_db_id++;
    }

    aiDetachAllLogStreams();
  }

  virtual void
  parameterCallbackJsonDb(const std::string& json_db) {
    *json_db_ = json_db;
    if (json_db_->empty())
      return;

    object_recognition_core::db::ObjectDbParameters parameters(*json_db_);

    if (parameters.type() == object_recognition_core::db::ObjectDbParameters::NONCORE) {
      // If we are dealing with a household DB
      db_.reset(new ObjectDbSqlHousehold());
      db_->set_parameters(parameters);
    } else {
      // If we are dealing with an ORK DB
      if (!db_)
        db_ = ObjectDbParameters(*json_db_).generateDb();
      parameterCallbackCommon();
    }
  }

  void
  parameterCallbackModelSet(const std::string& model_set) {
    //std::vector<object_recognition_core::db::ModelId> object_ids;

    //boost::python::stl_input_iterator<std::string> begin(python_object_ids), end;
    //std::copy(begin, end, std::back_inserter(object_ids));

    object_recognition_core::db::ObjectDbParameters parameters(*json_db_);

    if (parameters.type() != object_recognition_core::db::ObjectDbParameters::NONCORE)
      return;

    object_recognizer_ = tabletop_object_detector::TabletopObjectRecognizer();

    boost::shared_ptr<household_objects_database::ObjectsDatabase> database = dynamic_cast<ObjectDbSqlHousehold*>(&(*db_))->db();

    std::vector<boost::shared_ptr<household_objects_database::DatabaseScaledModel> > models;
    std::cout << "Loading model set: " << model_set << std::endl;
    if (!database->getScaledModelsBySet(models, model_set))
      return;

    object_recognizer_.clearObjects();
    for (size_t i = 0; i < models.size(); i++) {
      int model_id = models[i]->id_.data();
      shape_msgs::Mesh mesh;

      std::cout << "Loading model: " << model_id;
      if (!database->getScaledModelMesh(model_id, mesh)) {
        std::cout << "  ... Failed" << std::endl;
        continue;
      }

      object_recognizer_.addObject(model_id, mesh);
      std::stringstream ss;
      ss << model_id;
      household_id_to_db_id_[model_id] = ss.str();
      std::cout << std::endl;
    }

  }

  static void declare_params(ecto::tendrils& params) {
    object_recognition_core::db::bases::declare_params_impl(params, "mesh");
    params.declare(&ObjectRecognizer::tabletop_object_ids_, "tabletop_object_ids",
                   "The object_ids set as defined by the household object database.",
                   "REDUCED_MODEL_SET");
  }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&ObjectRecognizer::clusters_, "clusters3d", "The object clusters.").required(true);
      inputs.declare(&ObjectRecognizer::table_coefficients_, "table_coefficients", "The coefficients of planar surfaces.").required(true);

      outputs.declare(&ObjectRecognizer::pose_results_, "pose_results", "The results of object recognition");
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      configure_impl();

      tabletop_object_ids_.set_callback(boost::bind(&ObjectRecognizer::parameterCallbackModelSet, this, _1));
      tabletop_object_ids_.dirty(true);

      perform_fit_merge_ = true;
      confidence_cutoff_ = 0.85f;
    }

    /** Compute the pose of the table plane
     * @param inputs
     * @param outputs
     * @return
     */
    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      std::vector<tabletop_object_detector::TabletopObjectRecognizer::TabletopResult > results;

      // Process each table
      std::vector<std::vector<cv::Vec3f> > clusters_merged;
      clusters_merged.reserve(100);
      std::vector<size_t> cluster_table;
      cluster_table.reserve(100);

      std::vector<cv::Vec3f> translations(clusters_->size());
      std::vector<cv::Matx33f> rotations(clusters_->size());
      for (size_t table_index = 0; table_index < clusters_->size(); ++table_index)
      {
        getPlaneTransform((*table_coefficients_)[table_index], rotations[table_index], translations[table_index]);

        cv::Matx33f Rinv = rotations[table_index].t();
        cv::Vec3f Tinv = -Rinv*translations[table_index];

      BOOST_FOREACH(const std::vector<cv::Vec3f>& cluster, (*clusters_)[table_index]) {
        clusters_merged.resize(clusters_merged.size() + 1);
        for (size_t i = 0; i < cluster.size(); ++i) {
          cv::Vec3f res = Rinv * cluster[i] + Tinv;
          clusters_merged.back().push_back(cv::Vec3f(res[0], res[1], res[2]));
        }
        cluster_table.push_back(table_index);
      }
    }

      // Find possible candidates
      object_recognizer_.objectDetection(clusters_merged, confidence_cutoff_, perform_fit_merge_, results);

      // Define the results
      pose_results_->clear();
      for (size_t i = 0; i < results.size(); ++i)
      {
        const tabletop_object_detector::TabletopObjectRecognizer::TabletopResult & result = results[i];
        const size_t table_index = cluster_table[result.cloud_index_];

        PoseResult pose_result;

        // Add the object id
        std::string object_id = household_id_to_db_id_[result.object_id_];
        pose_result.set_object_id(db_, object_id);

        // Add the pose
        const geometry_msgs::Pose &pose = result.pose_;

        cv::Vec3f T(pose.position.x, pose.position.y, pose.position.z);
        T[2] -= min_z_[object_id];
        Eigen::Quaternionf quat(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

        cv::Vec3f new_T = rotations[table_index] * T + translations[table_index];
        pose_result.set_T(cv::Mat(new_T));

        pose_result.set_R(quat);
        cv::Mat R = cv::Mat(rotations[table_index] * pose_result.R<cv::Matx33f>());
        pose_result.set_R(R);
        pose_result.set_confidence(result.confidence_);

        // Add the cluster of points
        std::vector<sensor_msgs::PointCloud2Ptr> ros_clouds (1);

        ros_clouds[0].reset(new sensor_msgs::PointCloud2());
        sensor_msgs::PointCloud2Proxy<sensor_msgs::PointXYZ> proxy(*(ros_clouds[0]));

      cv::Matx33f Rot = rotations[table_index];
      cv::Vec3f Tra = translations[table_index];
      // Add the cloud
      proxy.resize(result.cloud_.size());
      sensor_msgs::PointXYZ *iter = &(proxy[0]);
      for(size_t i = 0; i < result.cloud_.size(); ++i, ++iter) {
        //Transform the object points back to their original frame
        cv::Vec3f res = Rot * result.cloud_[i] + Tra;
        iter->x = res[0];
        iter->y = res[1];
        iter->z = res[2];
      }

        pose_result.set_clouds(ros_clouds);

        pose_results_->push_back(pose_result);
      }
      return ecto::OK;
    }

  private:
    typedef std::vector<tabletop_object_detector::ModelFitInfo> ModelFitInfos;
    /** The object recognizer */
    tabletop_object_detector::TabletopObjectRecognizer object_recognizer_;
    /** The resulting poses of the objects */
    ecto::spore<std::vector<PoseResult> > pose_results_;
    /** The input clusters */
    ecto::spore<std::vector<std::vector<std::vector<cv::Vec3f> > > > clusters_;
    /** The coefficients of the tables */
    ecto::spore<std::vector<cv::Vec4f> > table_coefficients_;
    /** The number of models to fit to each cluster */
    float confidence_cutoff_;
    bool perform_fit_merge_;
    ecto::spore<std::string> tabletop_object_ids_;
  /** map to convert from artificial household id to db id */
  std::map<size_t, std::string> household_id_to_db_id_;
  /** for each DB id, store the minimum z */
  std::map<std::string, double> min_z_;
};
}

ECTO_CELL(tabletop_object, tabletop::ObjectRecognizer, "ObjectRecognizer",
          "Given clusters on a table, identify them as objects.")
