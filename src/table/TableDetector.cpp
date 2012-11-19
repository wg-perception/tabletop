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

//#if PCL_VERSION_COMPARE(>=,1,6,0)
#if 0
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
#endif

#include <tf/transform_listener.h>

using ecto::tendrils;

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
  struct TableDetector
  {
    static void
    declare_params(ecto::tendrils& params)
    {
      float c_limits[6] =
      { -std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
      std::vector<float> limits(c_limits, c_limits + 6);
      params.declare(&TableDetector::filter_limits_, "filter_limits",
                     "The limits of the interest box to find a table, in order [xmin,xmax,ymin,ymax,zmin,zmax]",
                     limits);
      params.declare(&TableDetector::min_table_size_, "min_table_size",
                     "The minimum number of points deemed necessary to find a table.", 10000);
      params.declare(&TableDetector::plane_detection_voxel_size_, "plane_detection_voxel_size",
                     "The size of a voxel cell when downsampling ", 0.01);
      params.declare(&TableDetector::normal_k_search_, "normal_k_search",
                     "The number of nearest neighbors to use when computing normals", 10);
      params.declare(&TableDetector::plane_threshold_, "plane_threshold",
                     "The distance used as a threshold when finding a plane", 0.1);
      params.declare(&TableDetector::table_cluster_tolerance_, "table_cluster_tolerance",
                     "The distance used when clustering a plane", 0.2);
      Eigen::Vector3f default_up(0, 0, 1);
      params.declare(&TableDetector::up_direction_, "vertical_direction", "The vertical direction", default_up);
      params.declare(&TableDetector::up_frame_id_, "vertical_frame_id", "The vertical frame id", "/map");
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&TableDetector::cloud_in_, "cloud", "The point cloud in which to find a table.");
      inputs.declare(&TableDetector::flatten_plane_, "flatten_plane",
                     "If true, the plane's normal is vertical_direction.", false);

      outputs.declare(&TableDetector::table_coefficients_, "coefficients", "The coefficients of planar surfaces.");
      outputs.declare(&TableDetector::table_rotations_, "rotations", "The pose rotations of the tables.");
      outputs.declare(&TableDetector::table_translations_, "translations", "The pose translations of the tables");
      outputs.declare(&TableDetector::clouds_out_, "clouds", "Samples that belong to the table.");
      outputs.declare(&TableDetector::clouds_hull_, "clouds_hull", "Hulls of the samples.");
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
      TabletopSegmenter table_segmenter(*filter_limits_, *min_table_size_, *plane_detection_voxel_size_,
                                        *normal_k_search_, *plane_threshold_, *table_cluster_tolerance_);
      pcl::ModelCoefficients::Ptr table_coefficients;

      // Find the table, it assumes only one plane for now TODO
      clouds_out_->clear();
      clouds_hull_->clear();
      table_coefficients_->clear();

//#if PCL_VERSION_COMPARE(>=,1,6,0)
#if 0
      pcl::PointCloud<PointT>::Ptr init_cloud_ptr(new pcl::PointCloud<PointT>);
      pcl::PointCloud<PointT>::Ptr prev_cloud(new pcl::PointCloud<PointT>);
      *prev_cloud = *(*cloud_in_);

      pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
      ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
      ne.setMaxDepthChangeFactor(0.03f);
      ne.setNormalSmoothingSize(20.0f);

      pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
      mps.setMinInliers(*min_table_size_);
      mps.setAngularThreshold(0.017453 * 2.0); //3 degrees
      mps.setDistanceThreshold(*plane_threshold_);// from params

      std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
      pcl::PointCloud<PointT>::Ptr contour(new pcl::PointCloud<PointT>);
      size_t prev_models_size = 0;

      regions.clear();
      pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
      ne.setInputCloud(prev_cloud);
      ne.compute(*normal_cloud);

      mps.setInputNormals(normal_cloud);
      mps.setInputCloud(prev_cloud);
      mps.segmentAndRefine(regions);
      BOOST_FOREACH(const pcl::PlanarRegion<PointT> & region, regions)
      table_coefficients_->push_back(region.getCoefficients());
#else
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull;
      pcl::PointCloud<PointT>::Ptr cloud_copy(new pcl::PointCloud<PointT>);
      *cloud_copy = *(*cloud_in_);

      while (table_segmenter.findTable<pcl::PointXYZ>(cloud_copy, table_coefficients, cloud_out, cloud_hull)
             == TabletopSegmenter::SUCCESS)
      {
        // First of all, check that the table has a normal close to what is wanted
        //(*cloud_in_)->header;
        if (!up_frame_id_->empty())
        {
          Eigen::Vector4f up_direction(table_coefficients->values[0], table_coefficients->values[1],
                                       table_coefficients->values[2], table_coefficients->values[3]);
          /*tf::TransformListener listener;
          geometry_msgs::Vector3Stamped stamped_in;
          geometry_msgs::Vector3Stamped stamped_out;
          listener.tranformVector(*up_frame_id_, stamped_in, stamped_out);*/
        }

        //
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out_copy(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull_copy(new pcl::PointCloud<pcl::PointXYZ>);
        *cloud_out_copy = *cloud_out;
        *cloud_hull_copy = *cloud_hull;
        clouds_out_->push_back(cloud_out_copy);
        clouds_hull_->push_back(cloud_hull_copy);
        table_coefficients_->push_back(
            Eigen::Vector4f(table_coefficients->values[0], table_coefficients->values[1], table_coefficients->values[2],
                            table_coefficients->values[3]));

        // Remove the table from the current point cloud
        pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism_;

        // ---[ Get the objects on top of the (non-flat) table
        pcl::PointIndices::Ptr cloud_object_indices(new pcl::PointIndices);
        //prism_.setInputCloud (cloud_all_minus_table_ptr);
        prism_.setInputCloud(cloud_copy);
        prism_.setInputPlanarHull(cloud_hull);
        prism_.setHeightLimits(-10000, 10000);
        prism_.segment(*cloud_object_indices);

        pcl::ExtractIndices<pcl::PointXYZ> extractor;
        extractor.setInputCloud(cloud_copy);
        extractor.setIndices(cloud_object_indices);
        extractor.setNegative(true);
        extractor.filter(*cloud_copy);
      }
#endif

      // Compute the corresponding poses
      table_rotations_->resize(table_coefficients_->size());
      table_translations_->resize(table_coefficients_->size());
      for (size_t i = 0; i < table_coefficients_->size(); ++i)
        getPlaneTransform((*table_coefficients_)[i], *up_direction_, *flatten_plane_, (*table_translations_)[i],
                          (*table_rotations_)[i]);

      return ecto::OK;
    }
  private:
    /** The limits of the interest box to find a table, in order [xmin,xmax,ymin,ymax,zmin,zmax] */
    ecto::spore<std::vector<float> > filter_limits_;
    /** The minimum number of points deemed necessary to find a table */
    ecto::spore<size_t> min_table_size_;
    /** The size of a voxel cell when downsampling */
    ecto::spore<float> plane_detection_voxel_size_;
    /** The number of nearest neighbors to use when computing normals */
    ecto::spore<unsigned int> normal_k_search_;
    /** The distance used as a threshold when finding a plane */
    ecto::spore<float> plane_threshold_;

    /** if true, the plane coefficients are modified so that up_direction_in is the normal */
    ecto::spore<bool> flatten_plane_;
    /** The input cloud */
    ecto::spore<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> cloud_in_;
    /** The input cloud */
    ecto::spore<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> > clouds_out_;
    /** The output cloud */
    ecto::spore<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> > clouds_hull_;
    /** The rotations of the tables */
    ecto::spore<std::vector<Eigen::Matrix3f> > table_rotations_;
    /** The translations of the tables */
    ecto::spore<std::vector<Eigen::Vector3f> > table_translations_;
    /** The minimum number of inliers in order to do pose matching */
    ecto::spore<std::vector<Eigen::Vector4f> > table_coefficients_;
    /** The vertical direction */
    ecto::spore<Eigen::Vector3f> up_direction_;
    /** The frame id of the vertical direction */
    ecto::spore<std::string> up_frame_id_;

    ecto::spore<float> table_cluster_tolerance_;
  };
}

ECTO_CELL(tabletop_table, tabletop::TableDetector, "TableDetector", "Given a point cloud, find  a potential table.");
