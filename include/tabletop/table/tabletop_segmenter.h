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

#ifndef TABLETOP_SEGMENTER_H_
#define TABLETOP_SEGMENTER_H_

#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace tabletop
{
  template<typename Point>
  class TabletopSegmenter
  {
  public:
    enum Result
    {
      NO_CLOUD_RECEIVED, NO_TABLE, OTHER_ERROR, SUCCESS
    };

    TabletopSegmenter(const std::vector<float> & filter_limits, size_t min_cluster_size,
                      float plane_detection_voxel_size, unsigned int normal_k_search, float plane_threshold)
        :
          filter_limits_(filter_limits),
          min_cluster_size_(min_cluster_size),
          plane_detection_voxel_size_(plane_detection_voxel_size),
          normal_k_search_(normal_k_search),
          plane_threshold_(plane_threshold)
    {
    }

    /** Filter a point cloud by removing points that are not within the bounding box
     * @param cloud_in
     * @param filter_limits limits of the box in order [xmin,xmax,ymin,ymax,zmin,zmax]
     * @param cloud_out
     */
    static
    void
    filterLimits(const typename pcl::PointCloud<Point>::ConstPtr & cloud_in, const std::vector<float> & filter_limits,
                 typename pcl::PointCloud<Point>::Ptr &cloud_out)
    {
      if (filter_limits.empty())
      {
        filterNaNs(cloud_in, cloud_out);
        return;
      }

      pcl::PassThrough<Point> pass_filter;
      typename pcl::PointCloud<Point>::Ptr z_cloud_filtered_ptr(new pcl::PointCloud<Point>), y_cloud_filtered_ptr(
          new pcl::PointCloud<Point>);

      pass_filter.setInputCloud(cloud_in);
      pass_filter.setFilterFieldName("z");
      pass_filter.setFilterLimits(filter_limits[4], filter_limits[5]);
      pass_filter.filter(*z_cloud_filtered_ptr);

      pass_filter.setInputCloud(z_cloud_filtered_ptr);
      pass_filter.setFilterFieldName("y");
      pass_filter.setFilterLimits(filter_limits[2], filter_limits[3]);
      pass_filter.filter(*y_cloud_filtered_ptr);

      pass_filter.setInputCloud(y_cloud_filtered_ptr);
      pass_filter.setFilterFieldName("x");
      pass_filter.setFilterLimits(filter_limits[0], filter_limits[1]);
      pass_filter.filter(*cloud_out);
    }

    /** Filter a point cloud by removing the NaNs
     * @param cloud_in
     * @param cloud_out
     */
    static
    void
    filterNaNs(const typename pcl::PointCloud<Point>::ConstPtr & cloud_in,
               typename pcl::PointCloud<Point>::Ptr &cloud_out)
    {
      pcl::PassThrough<Point> pass_filter;

      pass_filter.setInputCloud(cloud_in);
      pass_filter.setFilterFieldName("z");
      pass_filter.setFilterLimits(0, std::numeric_limits<float>::max());
      pass_filter.filter(*cloud_out);
    }

    void
    downsample(float downLeafSize, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
               pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out)
    {
      pcl::VoxelGrid<pcl::PointXYZ> downsampler;
      downsampler.setDownsampleAllData(false);
      downsampler.setLeafSize(downLeafSize, downLeafSize, downLeafSize);

      downsampler.setInputCloud(cloud_in);
      downsampler.setLeafSize(downLeafSize, downLeafSize, downLeafSize);
      downsampler.filter(*cloud_out);
    }

    void
    estimateNormals(unsigned int normal_k_search, const typename pcl::PointCloud<Point>::Ptr &cloud,
                    typename pcl::PointCloud<pcl::Normal>::Ptr &normals)
    {
      pcl::NormalEstimation<Point, pcl::Normal> normalsEstimator;
      normalsEstimator.setInputCloud(cloud);
#if PCL_MINOR_VERSION<=3
      typename pcl::KdTree<Point>::Ptr tree(new pcl::KdTreeFLANN<Point>());
#else
      typename pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>());
#endif
      normalsEstimator.setSearchMethod(tree);
      normalsEstimator.setKSearch(normal_k_search);
      normalsEstimator.compute(*normals);
    }

    bool
    segmentPlane(float distanceThreshold, const typename pcl::PointCloud<Point>::Ptr &cloud,
                 const typename pcl::PointCloud<pcl::Normal>::Ptr &normals, pcl::PointIndices::Ptr &inliers,
                 pcl::ModelCoefficients::Ptr &coefficients)
    {
      pcl::SACSegmentationFromNormals<Point, pcl::Normal> tableSegmentator;
      // Table model fitting parameters
      tableSegmentator.setDistanceThreshold(distanceThreshold);
      tableSegmentator.setMaxIterations(10000);
      tableSegmentator.setNormalDistanceWeight(0.1);
      tableSegmentator.setOptimizeCoefficients(true);
      tableSegmentator.setModelType(pcl::SACMODEL_NORMAL_PLANE);
      tableSegmentator.setMethodType(pcl::SAC_RANSAC);
      tableSegmentator.setProbability(0.99);

      tableSegmentator.setInputCloud(cloud);
      tableSegmentator.setInputNormals(normals);
      tableSegmentator.segment(*inliers, *coefficients);

      return !inliers->indices.empty();
    }

    Result
    findTable(const typename pcl::PointCloud<Point>::ConstPtr & cloud_in, typename pcl::PointIndices::Ptr table_inliers_ptr,
              typename pcl::ModelCoefficients::Ptr table_coefficients_ptr)
    {
      // First, filter by an interest box (which also remove NaN's)
      cloud_filtered_ptr_ = typename pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>);
      filterLimits(cloud_in, filter_limits_, cloud_filtered_ptr_);

      if (cloud_filtered_ptr_->points.size() < min_cluster_size_)
      {
        // TODO
        //ROS_INFO("Filtered cloud only has %d points", (int)cloud_filtered_ptr->points.size());
        return NO_TABLE;
      }

      // Then, downsample
      cloud_downsampled_ptr_ = typename pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>);
      downsample(plane_detection_voxel_size_, cloud_filtered_ptr_, cloud_downsampled_ptr_);
      if (cloud_downsampled_ptr_->points.size() < min_cluster_size_)
      {
        //ROS_INFO("Downsampled cloud only has %d points", (int)cloud_downsampled_ptr->points.size());
        return NO_TABLE;
      }

      // Estimate the normals
      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_ptr(new pcl::PointCloud<pcl::Normal>);
      estimateNormals(normal_k_search_, cloud_downsampled_ptr_, cloud_normals_ptr);

      // Perform planar segmentation
      if (!segmentPlane(plane_threshold_, cloud_downsampled_ptr_, cloud_normals_ptr, table_inliers_ptr,
                   table_coefficients_ptr))
        return NO_TABLE;

      if (table_coefficients_ptr->values.size() <= 3)
      {
        //ROS_INFO("Failed to detect table in scan");
        return NO_TABLE;
      }
      /*
       if (table_inliers_ptr->indices.size() < (unsigned int) inlier_threshold_)
       {
       //ROS_INFO(
       //  "Plane detection has %d inliers, below min threshold of %d", (int)table_inliers_ptr->indices.size(), inlier_threshold_);
       return NO_TABLE;
       }
       */
      //ROS_INFO(
      //  "[TableObjectDetector::input_callback] Model found with %d inliers: [%f %f %f %f].", (int)table_inliers_ptr->indices.size (), table_coefficients_ptr->values[0], table_coefficients_ptr->values[1], table_coefficients_ptr->values[2], table_coefficients_ptr->values[3]);
      return SUCCESS;
    }

    typename pcl::PointCloud<Point>::Ptr cloud_filtered_ptr_;
    typename pcl::PointCloud<Point>::Ptr cloud_downsampled_ptr_;
  private:
    /** The limits of the interest box to find a table, in order [xmin,xmax,ymin,ymax,zmin,zmax] */
    std::vector<float> filter_limits_;
    /** The minimum number of points deemed necessary to find a table */
    size_t min_cluster_size_;
    /** The size of a voxel cell when downsampling */
    float plane_detection_voxel_size_;
    /** The number of nearest neighbors to use when computing normals */
    unsigned int normal_k_search_;
    /** The distance used as a threshold when finding a plane */
    float plane_threshold_;
  };
}

#endif /* TABLETOP_SEGMENTER_H_ */
