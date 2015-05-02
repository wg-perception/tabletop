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

#include <tabletop_object_detector/iterative_distance_fitter.h>

#include <boost/bind.hpp>

namespace tabletop_object_detector
{

//------------------------- Implementation follows ----------------------------------------

/*! Computes the point at the bottom of the point cloud vertical to the center of
 *  gravity. This is the point where the table supports the object.
 */
cv::Point3f IterativeTranslationFitter::centerOfSupport(const std::vector<cv::Vec3f>& cloud) const
{
  cv::Point3f center;
  center.x = center.y = center.z = 0;
  if (cloud.empty()) {
    return center;
  }
  for (unsigned int i = 0; i < cloud.size(); ++i) {
    center.x += cloud[i][0];
    center.y += cloud[i][1];
  }
  center.x /= cloud.size();
  center.y /= cloud.size();
  return center;
}


double IterativeTranslationFitter::getFitScoreAndGradient(const std::vector<cv::Vec3f>& cloud,
    const cv::Point3f& location, cv::Point3f& vector,
    boost::function<double(double)> kernel) const
{
  double inlier_count = 0;

  vector.x = 0;
  vector.y = 0;
  vector.z = 0;
  int cnt = 0;

  for (size_t i = 0; i < cloud.size(); i++) {
    double wx = cloud[i][0] - location.x;
    double wy = cloud[i][1] - location.y;
    double wz = cloud[i][2] - location.z;

    int x, y, z;
    double val = truncate_value_;
    if (distance_voxel_grid_->worldToGrid(wx, wy, wz, x, y, z)) {
      const distance_field::PropDistanceFieldVoxel& voxel = distance_voxel_grid_->getCell(x, y, z);
      double cx, cy, cz;
      if (voxel.closest_point_[0] != distance_field::PropDistanceFieldVoxel::UNINITIALIZED) {
        distance_voxel_grid_->gridToWorld(voxel.closest_point_[0],
                                          voxel.closest_point_[1],
                                          voxel.closest_point_[2],
                                          cx, cy, cz);
        val = distance_voxel_grid_->getDistance(x, y, z);
        double weight = kernel(val);
        vector.x += weight * (cx - wx);
        vector.y += weight * (cy - wy);
        vector.z += weight * (cz - wz);

        inlier_count += weight;
      }
    }
  }

  if (inlier_count != 0) {
    vector.x /=  inlier_count;
    vector.y /=  inlier_count;
    vector.z /=  inlier_count;
  }

  return inlier_count / cloud.size();
}

/*! Iterates over the inner loop of \a getFitScoreAndGradient, then moves in the direction
 *  of the computed gradient. Does this until the score stops decreasing.
 *
 *  The fit is initialized to the centroid of the cloud along x and y, and 0 along z. This
 *  assumes that the meshes are all such that the origin is at the bottom, with the z axis
 *  pointing up. It also assumes the points have been translated to a coordinate frame
 *  where z=0 is the table plane.
 *
 *  For the same reason. there is no iteration done along z at all.
 */
ModelFitInfo IterativeTranslationFitter::fitPointCloud(const std::vector<cv::Vec3f>& cloud,
    cv::flann::Index &search, double min_object_score) const
{
  if (cloud.empty()) {
    //ROS_ERROR("Attempt to fit model to empty point cloud");
    geometry_msgs::Pose bogus_pose;
    return ModelFitInfo(model_id_, bogus_pose, 0.0);
  }

  // compute center of point cloud
  cv::Point3f center = centerOfSupport(cloud);

  cv::Point3f location = center;
  cv::Point3f vector;
  geometry_msgs::Pose pose;

  const double clipping = 0.0075;
  boost::function<double(double)> kernel = boost::bind(huberKernel, clipping, _1);
  const int max_iterations = 100;
  int iter = 0;
  double score = 0;
  const double EPS = 0.0;

  do {
    double new_score = getFitScoreAndGradient(cloud, location, vector, kernel);
    if (new_score > score + EPS) {
      score = new_score;
      location.x -= vector.x;
      location.y -= vector.y;
    } else
      break;
  } while (++iter < max_iterations);


  if (iter == max_iterations) {
    //ROS_WARN("Maximum iterations reached in model fitter");
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
  if (score > min_object_score) {
    double model_score = getModelFitScore(cloud, location, kernel, search);
    // since for waterthight model only 50% of the points are visible at max, we weight the model_score only half.
    score *= sqrt(model_score);
  } else
    score = 0;

  return ModelFitInfo(model_id_, pose, score);
}

double IterativeTranslationFitter::getModelFitScore(const std::vector<cv::Vec3f>& cloud, const cv::Point3f& position,
    boost::function<double(double)> kernel,
    cv::flann::Index &search) const
{
  double inlier_count = 0;
  std::vector<int> indices(1);
  std::vector<float> distances(1);
  cv::Mat_<float> points(1, 3);
  for (std::vector<cv::Point3f>::const_iterator mIt = model_points_.begin(); mIt != model_points_.end(); ++mIt) {
    points(0, 0) = mIt->x + position.x;
    points(0, 1) = mIt->y + position.y;
    points(0, 2) = mIt->z + position.z;

    search.knnSearch(points, indices, distances, 1);
    inlier_count += kernel(sqrt(distances[0]));
  }
  return inlier_count / model_points_.size();
}

} //namespace
