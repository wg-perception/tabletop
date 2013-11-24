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

// Author(s): Marius Muja, Matei Ciocarlie and Romain Thibaux

#ifndef _ITERATIVE_DISTANCE_FITTER_H_
#define _ITERATIVE_DISTANCE_FITTER_H_

#include "tabletop_object_detector/model_fitter.h"
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <pcl/search/search.h>

#include <math.h>
#include <moveit/distance_field/propagation_distance_field.h>

namespace tabletop_object_detector {

// use M-kernel to weight inliers and suppress the influence of outliers from linear to just constant ->
// 1) more robust ICP (not too sensitive to outliers)
// 2) score = inliers, but as a floating point value -> no cut off threshold -> smooth -> can better distinguish between similar poses.
inline double huberKernel (double clipping, double x)
{
  if (x < clipping)
    return 1.0;
  else
    return (clipping / x);
}

//! Does an ICP-like fitting only in the X and Y translation DOFs
template<typename PointCloudType>
class IterativeTranslationFitter : public DistanceFieldFitter
{
 private:

  //! Helper function for fitting
  geometry_msgs::Point32 centerOfSupport(const PointCloudType& cloud) const;

  //! Inner loop when doing translation fitting
  double getFitScoreAndGradient(const PointCloudType& cloud,
				  const geometry_msgs::Point32& location, 
				  geometry_msgs::Point32& vector,
                  boost::function<double(double)> kernel) const;

  double getModelFitScore(const PointCloudType& cloud, const geometry_msgs::Point32& location,
                          boost::function<double(double)> kernel,
                          const pcl::search::Search<typename PointCloudType::PointType>& search) const;

 public:
  //! Stub, just calls super's constructor
  IterativeTranslationFitter() : DistanceFieldFitter() {}
  //! Empty stub
  ~IterativeTranslationFitter() {}

  //! Main fitting function
  ModelFitInfo fitPointCloud(const PointCloudType& cloud, const pcl::search::Search<typename PointCloudType::PointType>& search,
                             double min_object_score = 0.75) const;
};

//------------------------- Implementation follows ----------------------------------------

/*! Computes the point at the bottom of the point cloud vertical to the center of
  gravity. This is the point where the table supports the object. 
*/
template <class PointCloudType>
geometry_msgs::Point32 IterativeTranslationFitter<PointCloudType>::centerOfSupport(const PointCloudType& cloud) const
{
  geometry_msgs::Point32 center;
  center.x = center.y = center.z = 0;
  if (cloud.points.empty())
  {
    return center;
  }
  for (unsigned int i=0; i<cloud.points.size(); ++i) 
  {
    center.x += cloud.points[i].x;
    center.y += cloud.points[i].y;
  }
  center.x /= cloud.points.size();
  center.y /= cloud.points.size();
  return center;
}


template <class PointCloudType>
double IterativeTranslationFitter<PointCloudType>::getFitScoreAndGradient(const PointCloudType& cloud,
                              const geometry_msgs::Point32& location, geometry_msgs::Point32& vector,
                              boost::function<double(double)> kernel) const
{
  double inlier_count = 0;
  
  vector.x = 0;
  vector.y = 0;
  vector.z = 0;
  int cnt = 0;
  
  for (size_t i=0;i<cloud.points.size();i++) 
  {
    double wx = cloud.points[i].x-location.x;
    double wy = cloud.points[i].y-location.y;
    double wz = cloud.points[i].z-location.z;
    
    int x, y, z;
    double val = truncate_value_;
    if (distance_voxel_grid_->worldToGrid(wx,wy,wz,x,y,z)) 
    {
      const distance_field::PropDistanceFieldVoxel& voxel = distance_voxel_grid_->getCell(x,y,z);
      double cx, cy, cz;
      if (voxel.closest_point_[0] != distance_field::PropDistanceFieldVoxel::UNINITIALIZED) 
      {
        distance_voxel_grid_->gridToWorld(voxel.closest_point_[0],
					  voxel.closest_point_[1],
					  voxel.closest_point_[2],
					  cx,cy,cz);
        val = distance_voxel_grid_->getDistance(x,y,z);
        double weight = kernel (val);
        vector.x += weight * (cx-wx);
        vector.y += weight * (cy-wy);
        vector.z += weight * (cz-wz);

        inlier_count += weight;
      }
    }
  }

  if (inlier_count!=0)
  {
    vector.x /=  inlier_count;
    vector.y /=  inlier_count;
    vector.z /=  inlier_count;
  }

  return inlier_count / cloud.size();
}

/*! Iterates over the inner loop of \a getFitScoreAndGradient, then moves in the direction
  of the computed gradient. Does this until the score stops decreasing.

  The fit is initialized to the centroid of the cloud along x and y, and 0 along z. This 
  assumes that the meshes are all such that the origin is at the bottom, with the z axis 
  pointing up. It also assumes the points have been translated to a coordinate frame
  where z=0 is the table plane.

  For the same reason. there is no iteration done along z at all.
*/
template <class PointCloudType>
ModelFitInfo IterativeTranslationFitter<PointCloudType>::fitPointCloud(const PointCloudType& cloud,
                                                                       const pcl::search::Search<typename PointCloudType::PointType>& search,
                                                                       double min_object_score) const
{
  if (cloud.points.empty()) 
  {
    ROS_ERROR("Attempt to fit model to empty point cloud");
    geometry_msgs::Pose bogus_pose;
    return ModelFitInfo(model_id_, bogus_pose, 0.0);
  }
  
  // compute center of point cloud
  geometry_msgs::Point32 center = centerOfSupport(cloud);

  geometry_msgs::Point32 location = center;
  geometry_msgs::Point32 vector;
  geometry_msgs::Pose pose;

  const double clipping = 0.0075;
  boost::function<double(double)> kernel = boost::bind (huberKernel, clipping, _1);
  const int max_iterations = 100;
  int iter = 0;
  double score = 0;
  const double EPS = 0.0;

  do
  {
    double new_score = getFitScoreAndGradient(cloud, location, vector, kernel);
    if (new_score > score + EPS)
    {
      score = new_score;
      location.x -= vector.x;
      location.y -= vector.y;
    }
    else
      break;
  } while (++iter < max_iterations);


  if (iter == max_iterations) 
  {
    ROS_WARN("Maximum iterations reached in model fitter");
  }

  pose.position.x = location.x;
  pose.position.y = location.y;
  pose.position.z = location.z;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;

  // evaluating the model score is cost-intensive and since the model_score <= 1 ->
  // if score already below min_object_score, then set to 0 and stop further evaluation!
  if (score > min_object_score)
  {
    double model_score = getModelFitScore (cloud, location, kernel, search);
    // since for waterthight model only 50% of the points are visible at max, we weight the model_score only half.
    score *= sqrt (model_score);
  }
  else
    score = 0;

  return ModelFitInfo(model_id_, pose, score);
}

template <class PointCloudType>
double IterativeTranslationFitter<PointCloudType>::getModelFitScore(const PointCloudType& cloud, const geometry_msgs::Point32& position,
                                                    boost::function<double(double)> kernel,
                                                    const pcl::search::Search<typename PointCloudType::PointType>& search) const
{
  typedef typename PointCloudType::PointType PointT;
  double inlier_count = 0;
  std::vector<int> indices(1);
  std::vector<float> distances(1);
  PointT point;
  for (std::vector<tf::Vector3>::const_iterator mIt = model_points_.begin(); mIt != model_points_.end(); ++mIt)
  {
    point.x = mIt->x () + position.x;
    point.y = mIt->y () + position.y;
    point.z = mIt->z () + position.z;

    if (search.nearestKSearchT (point, 1, indices, distances) > 0)
      inlier_count += kernel (sqrt(distances[0]));
  }
  return inlier_count / model_points_.size();
}

} //namespace

#endif

