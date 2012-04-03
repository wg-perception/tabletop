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

#include <ecto/ecto.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tabletop/table/tabletop_segmenter.h>

using ecto::tendrils;

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/planar_polygon_fusion.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/plane_coefficient_comparator.h>
#include <pcl/segmentation/euclidean_plane_coefficient_comparator.h>
#include <pcl/segmentation/rgb_plane_coefficient_comparator.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>

using ecto::tendrils;

//typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;
typedef std::vector<pcl::PointCloud<PointT>, Eigen::aligned_allocator<pcl::PointCloud<PointT> > > CloudVectorType;

namespace tabletop
{
  /** Ecto implementation of a module that takes
   *
   */
  struct Clusterer
  {
    static void
    declare_params(ecto::tendrils& params)
    {
      params.declare(&Clusterer::clustering_voxel_size_, "clustering_voxel_size",
                     "The minimum number of points deemed necessary to find a table.", 0.003);
      params.declare(&Clusterer::cluster_distance_, "cluster_distance", "The size of a voxel cell when downsampling ",
                     0.01);
      params.declare(&Clusterer::min_cluster_size_, "min_cluster_size", "Min number of points for a cluster", 300);
      params.declare(&Clusterer::table_z_filter_min_, "table_z_filter_min",
                     "Min distance (in meters) from the table to get clusters from.", 0.01);
      params.declare(&Clusterer::table_z_filter_max_, "table_z_filter_max",
                     "Max distance (in meters) from the table to get clusters from.", 0.5);
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&Clusterer::cloud_, "cloud", "The point cloud in which to find the clusters.");
      inputs.declare(&Clusterer::clouds_hull_, "clouds_hull", "The hull in which to find the clusters.");

      outputs.declare(&Clusterer::clusters_, "clusters", "For each table, a vector of clusters.");
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
      BlobSegmenter blob_segmenter(*clustering_voxel_size_, *cluster_distance_, *min_cluster_size_,
                                   *table_z_filter_min_, *table_z_filter_max_);

      clusters_->clear();

#if PCL_VERSION_GE_160
//#if 0
      pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
      pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
      pcl::EdgeAwarePlaneComparator<PointT, pcl::Normal>::Ptr edge_aware_comparator_;
      pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr euclidean_cluster_comparator_;

      bool use_planar_refinement_ = true;
      bool use_clustering_;
      CloudVectorType prev_clusters_;

      // Estimate Normals
      pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      *cloud = *(*cloud_);
      ne.setInputCloud(cloud);
      ne.compute(*normal_cloud);
      float* distance_map = ne.getDistanceMap();
      boost::shared_ptr<pcl::EdgeAwarePlaneComparator<PointT, pcl::Normal> > eapc = boost::dynamic_pointer_cast<
          pcl::EdgeAwarePlaneComparator<PointT, pcl::Normal> >(edge_aware_comparator_);
      eapc->setDistanceMap(distance_map);

      // Segment Planes
      printf("Segmenting planes...\n");
      std::vector<pcl::PlanarRegion<PointT> > regions;
      std::vector<pcl::ModelCoefficients> model_coefficients;
      std::vector<pcl::PointIndices> inlier_indices;
      pcl::PointCloud<pcl::Label>::Ptr labels(new pcl::PointCloud<pcl::Label>);
      std::vector<pcl::PointIndices> label_indices;
      std::vector<pcl::PointIndices> boundary_indices;
      mps.setInputNormals(normal_cloud);
      mps.setInputCloud(cloud);
      if (use_planar_refinement_)
      {
        mps.segmentAndRefine(regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
      }
      else
      {
        mps.segment(regions); //, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
      }

      //Segment Objects
      CloudVectorType clusters;

      if (use_clustering_ && regions.size() > 0)
      {
        std::vector<bool> plane_labels;
        plane_labels.resize(label_indices.size(), false);
        for (size_t i = 0; i < label_indices.size(); i++)
        {
          if (label_indices[i].indices.size() > 10000)
          {
            plane_labels[i] = true;
          }
        }

        euclidean_cluster_comparator_->setInputCloud(cloud);
        euclidean_cluster_comparator_->setLabels(labels);
        euclidean_cluster_comparator_->setExcludeLabels(plane_labels);

        pcl::PointCloud < pcl::Label > euclidean_labels;
        std::vector<pcl::PointIndices> euclidean_label_indices;
        pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> euclidean_segmentation(
            euclidean_cluster_comparator_);
        euclidean_segmentation.setInputCloud(cloud);
        euclidean_segmentation.segment(euclidean_labels, euclidean_label_indices);

        clusters_->resize(1);
        for (size_t i = 0; i < euclidean_label_indices.size(); i++)
        {
          if (euclidean_label_indices[i].indices.size() > 1000)
          {
            pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
            pcl::copyPointCloud(*cloud, euclidean_label_indices[i].indices, *cluster);
            (*clusters_)[0].push_back(cluster);
          }
        }
      }
#else
      for (size_t table_index = 0; table_index < clouds_hull_->size(); ++table_index)
      {
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
        blob_segmenter.process<pcl::PointXYZ>(*cloud_, (*clouds_hull_)[table_index], clusters);

        clusters_->push_back(clusters);
      }
#endif

      return ecto::OK;
    }
  private:
    /** Size of downsampling grid before performing clustering */
    ecto::spore<float> clustering_voxel_size_;
    /** Min distance between two clusters */
    ecto::spore<float> cluster_distance_;
    /** Min number of points for a cluster */
    ecto::spore<int> min_cluster_size_;
    /** Limits used when clustering points off the plane */
    ecto::spore<float> table_z_filter_min_;
    ecto::spore<float> table_z_filter_max_;

    /** The input cloud */
    ecto::spore<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> cloud_;
    /** The hull of the input cloud */
    ecto::spore<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> > clouds_hull_;
    /** The resulting clusters: for each table, return a vector of clusters */
    ecto::spore<std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> > > clusters_;
  };
}

ECTO_CELL(tabletop_table, tabletop::Clusterer, "Clusterer",
          "Given a point cloud and the hull of the table, find clusters.");
