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
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>

namespace tabletop
{
  /** Assumes plane coefficients are of the form ax+by+cz+d=0, normalized
   * @param plane_coefficients
   * @param up_direction_in
   * @param flatten_plane if true, the plane coefficients are modified so that up_direction_in is the normal
   */
  void
  getPlaneTransform(const Eigen::Vector4f &plane_coefficients, const Eigen::Vector3f &up_direction, bool flatten_plane,
                    Eigen::Vector3f & translation, Eigen::Matrix3f & rotation);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  template<typename Point>
  struct PointCloudView
  {
    typename pcl::PointCloud<Point>::ConstPtr & cloud_;
    typename pcl::PointIndices::ConstPtr indices_;
  };

  class TabletopSegmenter
  {
  public:
    enum Result
    {
      NO_CLOUD_RECEIVED, NO_TABLE, OTHER_ERROR, SUCCESS
    };

    TabletopSegmenter(const std::vector<float> & filter_limits, size_t min_cluster_size,
                      float plane_detection_voxel_size, unsigned int normal_k_search, float plane_threshold,
                      float cluster_tolerance)
        :
          filter_limits_(filter_limits),
          min_cluster_size_(min_cluster_size),
          plane_detection_voxel_size_(plane_detection_voxel_size),
          normal_k_search_(normal_k_search),
          plane_threshold_(plane_threshold),
          cluster_tolerance_(cluster_tolerance)
    {
    }

    /** Filter a point cloud by removing points that are not within the bounding box
     * @param cloud_in
     * @param filter_limits limits of the box in order [xmin,xmax,ymin,ymax,zmin,zmax]
     * @param cloud_out
     */
    template<typename Point>
    static
    void
    filterLimits(const typename pcl::PointCloud<Point>::ConstPtr & cloud_in, const std::vector<float> & filter_limits,
                 typename pcl::PointCloud<Point>::Ptr &cloud_out)
    {
      if (filter_limits.empty())
      {
        filterNaNs<Point>(cloud_in, cloud_out);
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
    template<typename Point>
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

    template<typename Point>
    static
    void
    downsample(float downLeafSize, const typename pcl::PointCloud<Point>::Ptr &cloud_in,
               typename pcl::PointCloud<Point>::Ptr &cloud_out)
    {
      pcl::VoxelGrid<Point> downsampler;
      downsampler.setDownsampleAllData(false);
      downsampler.setLeafSize(downLeafSize, downLeafSize, downLeafSize);

      downsampler.setInputCloud(cloud_in);
      downsampler.setLeafSize(downLeafSize, downLeafSize, downLeafSize);
      downsampler.filter(*cloud_out);
    }

    template<typename Point>
    static
    void
    estimateNormals(unsigned int normal_k_search, const typename pcl::PointCloud<Point>::Ptr &cloud,
                    typename pcl::PointCloud<pcl::Normal>::Ptr &normals)
    {
      pcl::NormalEstimation<Point, pcl::Normal> normalsEstimator;
      normalsEstimator.setInputCloud(cloud);
      typename pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>());
      normalsEstimator.setSearchMethod(tree);
      normalsEstimator.setKSearch(normal_k_search);
      normalsEstimator.compute(*normals);
    }

    template<typename Point>
    static
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

    template<typename Point>
    static
    void
    projectInliersOnTable(const typename pcl::PointCloud<Point>::ConstPtr &cloud, const pcl::PointIndices::Ptr &inliers,
                          const pcl::ModelCoefficients::Ptr &coefficients,
                          typename pcl::PointCloud<Point>::Ptr &projectedInliers)
    {
      typename pcl::ProjectInliers<Point> projector;
      projector.setModelType(pcl::SACMODEL_PLANE);
      projector.setInputCloud(cloud);
      projector.setIndices(inliers);
      projector.setModelCoefficients(coefficients);

      projectedInliers = typename pcl::PointCloud<Point>::Ptr(new typename pcl::PointCloud<Point>);
      projector.filter(*projectedInliers);
    }

    template<typename Point>
    static
    void
    extractPointCloud(const typename pcl::PointCloud<Point> &cloud, const pcl::PointIndices::ConstPtr &inliers,
                      typename pcl::PointCloud<Point> &extractedCloud)
    {
      pcl::ExtractIndices<Point> extractor;
      extractor.setInputCloud(cloud.makeShared());
      extractor.setIndices(inliers);
      extractor.setNegative(false);
      extractor.filter(extractedCloud);
    }

    template<typename Point>
    static
    void
    reconstructConvexHull(const typename pcl::PointCloud<Point> &projectedInliers,
                          typename pcl::PointCloud<Point> &tableHull)
    {
      typename pcl::ConvexHull<Point> hullReconstruntor;
      hullReconstruntor.setInputCloud(projectedInliers.makeShared());
      hullReconstruntor.reconstruct(tableHull);
    }

    template<typename Point>
    typename pcl::PointCloud<Point>::Ptr
    getBestHull(const typename pcl::PointCloud<Point>::ConstPtr & projected_inliers)
    {
      typename pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
      tree->setInputCloud(projected_inliers);

      std::vector<pcl::PointIndices> clusterIndices;
      typename pcl::EuclideanClusterExtraction<Point> ec;
      ec.setClusterTolerance(cluster_tolerance_);
      ec.setSearchMethod(tree);
      ec.setInputCloud(projected_inliers);
      ec.extract(clusterIndices);

      int maxClusterIndex = 0;
      for (size_t i = 1; i < clusterIndices.size(); ++i)
      {
        if (clusterIndices[maxClusterIndex].indices.size() < clusterIndices[i].indices.size())
        {
          maxClusterIndex = i;
        }
      }

      pcl::PointCloud<Point> table;
      extractPointCloud(*projected_inliers, boost::make_shared<pcl::PointIndices>(clusterIndices[maxClusterIndex]),
                        table);

      typename pcl::PointCloud<Point>::Ptr table_hull(new typename pcl::PointCloud<Point>);
      reconstructConvexHull(table, *table_hull);

      return table_hull;
    }

    template<typename Point>
    Result
    findTable(const typename pcl::PointCloud<Point>::ConstPtr & cloud_in,
              typename pcl::ModelCoefficients::Ptr &table_coefficients_ptr,
              typename pcl::PointCloud<Point>::Ptr &table_projected_ptr,
              typename pcl::PointCloud<Point>::Ptr &table_hull_ptr)
    {
      table_coefficients_ptr = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);

      // First, filter by an interest box (which also remove NaN's)
      typename pcl::PointCloud<Point>::Ptr cloud_filtered_ptr = typename pcl::PointCloud<Point>::Ptr(
          new pcl::PointCloud<Point>);
      filterLimits<Point>(cloud_in, filter_limits_, cloud_filtered_ptr);

      if (cloud_filtered_ptr->points.size() < min_cluster_size_)
      {
        // TODO
        //ROS_INFO("Filtered cloud only has %d points", (int)cloud_filtered_ptr->points.size());
        return NO_TABLE;
      }

      // Then, downsample
      typename pcl::PointCloud<Point>::Ptr cloud_downsampled_ptr(new typename pcl::PointCloud<Point>);
      downsample<Point>(plane_detection_voxel_size_, cloud_filtered_ptr, cloud_downsampled_ptr);
      if (cloud_downsampled_ptr->points.size() < min_cluster_size_)
      {
        //ROS_INFO("Downsampled cloud only has %d points", (int)cloud_downsampled_ptr->points.size());
        return NO_TABLE;
      }

      // Estimate the normals
      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_ptr(new pcl::PointCloud<pcl::Normal>);
      estimateNormals<Point>(normal_k_search_, cloud_downsampled_ptr, cloud_normals_ptr);

      // Perform planar segmentation
      pcl::PointIndices::Ptr table_inliers_ptr(new pcl::PointIndices);
      if (!segmentPlane<Point>(plane_threshold_, cloud_downsampled_ptr, cloud_normals_ptr, table_inliers_ptr,
                               table_coefficients_ptr))
        return NO_TABLE;

      if (table_coefficients_ptr->values.size() <= 3)
      {
        //ROS_INFO("Failed to detect table in scan");
        return NO_TABLE;
      }

      // Project the inliers on the table
      projectInliersOnTable<Point>(cloud_downsampled_ptr, table_inliers_ptr, table_coefficients_ptr,
                                   table_projected_ptr);

      // Get the final hull
      table_hull_ptr = getBestHull<Point>(table_projected_ptr);

      return SUCCESS;
    }
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
    /** The cluster tolerance when calling EuclideanClusterExtraction */
    float cluster_tolerance_;
  };

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  class BlobSegmenter
  {
  public:
    BlobSegmenter(double clustering_voxel_size, double cluster_distance, int min_cluster_size)
        :
          clustering_voxel_size_(clustering_voxel_size),
          cluster_distance_(cluster_distance),
          min_cluster_size_(min_cluster_size)
    {
    }

    template<typename Point>
    void
    process(typename pcl::PointCloud<Point>::ConstPtr cloud, typename pcl::PointCloud<Point>::ConstPtr table_hull_ptr,
            std::vector<typename pcl::PointCloud<Point>::Ptr> & clusters)
    {
      typename pcl::ExtractPolygonalPrismData<Point> prism_;

      // ---[ Get the objects on top of the (non-flat) table
      pcl::PointIndices cloud_object_indices;
      //prism_.setInputCloud (cloud_all_minus_table_ptr);
      prism_.setInputCloud(cloud);
      prism_.setInputPlanarHull(table_hull_ptr);
      prism_.setHeightLimits(std::numeric_limits<float>::min(), std::numeric_limits<float>::max());
      prism_.segment(cloud_object_indices);

      typename pcl::PointCloud<Point>::Ptr cloud_objects_ptr(new pcl::PointCloud<Point>);
      pcl::ExtractIndices<Point> extract_object_indices;
      extract_object_indices.setInputCloud(cloud);
      extract_object_indices.setIndices(boost::make_shared<const pcl::PointIndices>(cloud_object_indices));
      extract_object_indices.filter(*cloud_objects_ptr);

      if (cloud_objects_ptr->points.empty())
      {
        return;
      }

      // ---[ Downsample the points
      pcl::VoxelGrid<Point> grid_objects_;
      grid_objects_.setLeafSize(clustering_voxel_size_, clustering_voxel_size_, clustering_voxel_size_);
      grid_objects_.setDownsampleAllData(false);

      typename pcl::PointCloud<Point>::Ptr cloud_objects_downsampled_ptr(new pcl::PointCloud<Point>);
      grid_objects_.setInputCloud(cloud_objects_ptr);
      grid_objects_.filter(*cloud_objects_downsampled_ptr);

      // ---[ If flattening the table, adjust the points on the table to be straight also
      //TODOif(flatten_table_) straightenPoints<pcl::PointCloud<Point> >(*cloud_objects_downsampled_ptr,
      //table_plane_trans, table_plane_trans_flat);

      // ---[ Split the objects into Euclidean clusters
      std::vector<pcl::PointIndices> clusters2;

      //pcl_cluster_.setInputCloud (cloud_objects_ptr);
      pcl::EuclideanClusterExtraction<Point> pcl_cluster_;
      // Clustering parameters
      typename pcl::search::KdTree<Point>::Ptr clusters_tree = boost::make_shared<pcl::search::KdTree<Point> >();

      pcl_cluster_.setClusterTolerance(cluster_distance_);
      pcl_cluster_.setMinClusterSize(min_cluster_size_);
      pcl_cluster_.setSearchMethod(clusters_tree);

      pcl_cluster_.setInputCloud(cloud_objects_downsampled_ptr);
      pcl_cluster_.extract(clusters2);

      // ---[ Convert clusters into the PointCloud message
      getClustersFromPointCloud2<Point>(*cloud_objects_downsampled_ptr, clusters2, clusters);
    }
  private:

    template<typename Point> void
    getClustersFromPointCloud2(const typename pcl::PointCloud<Point> &cloud_objects,
                               const std::vector<pcl::PointIndices> &clusters2,
                               std::vector<typename pcl::PointCloud<Point>::Ptr> &clusters)
    {
      clusters.resize(clusters2.size());
      for (size_t i = 0; i < clusters2.size(); ++i)
      {
        clusters[i]->points.resize(clusters2[i].indices.size());
        for (size_t j = 0; j < clusters[i]->points.size(); ++j)
        {
          clusters[i]->points[j].x = cloud_objects.points[clusters2[i].indices[j]].x;
          clusters[i]->points[j].y = cloud_objects.points[clusters2[i].indices[j]].y;
          clusters[i]->points[j].z = cloud_objects.points[clusters2[i].indices[j]].z;
        }
      }
    }
    //! Size of downsampling grid before performing clustering
    double clustering_voxel_size_;
    //! Min distance between two clusters
    double cluster_distance_;
    //! Min number of points for a cluster
    int min_cluster_size_;

  };
}

#endif /* TABLETOP_SEGMENTER_H_ */
