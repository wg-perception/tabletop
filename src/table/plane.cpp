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

/** This is an implementation of
 Fast Sampling Plane Filtering and Indoor Mobile Robots
 Joydeep Biswas, Manuela Veloso
 author: Vincent Rabaud
 */

#include <deque>
#include <numeric>
#include <set>
#include <string>

#include <boost/foreach.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/contrib/contrib.hpp>

#include <ecto/ecto.hpp>

using ecto::tendrils;

int
PointDistanceSq(const cv::Point2i & point_1, const cv::Point2i & point_2)
{
  return (point_1.x - point_2.x) * (point_1.x - point_2.x) + (point_1.y - point_2.y) * (point_1.y - point_2.y);
}

bool
CreatePoint(const cv::Point2i &point_in, const cv::Mat & points3d, const cv::Mat_<uchar> & overall_mask, int nu,
            cv::Point2i &point_out)
{
  static cv::RNG rng;
  while (true)
  {
    int dist_x = rng.uniform(nu, 3 * nu + 1);
    if (rng.uniform(0, 2) == 0)
      point_out.x = point_in.x - dist_x;
    else
      point_out.x = point_in.x + dist_x;
    if ((point_out.x >= 0) && (point_out.x < points3d.cols))
      break;
  }
  while (true)
  {
    int dist_y = rng.uniform(nu, 3 * nu + 1);
    if (rng.uniform(0, 2) == 0)
      point_out.y = point_in.y - dist_y;
    else
      point_out.y = point_in.y + dist_y;
    if ((point_out.y >= 0) && (point_out.y < points3d.rows))
      break;
  }

  return ((!cvIsNaN(points3d.at<float>(point_out.y, point_in.x, 0))) && (!overall_mask(point_out.y, point_out.x)));
}

/**
 *
 * @param x
 * @param y
 * @param width
 * @param height
 * @param cols
 * @param rows
 * @param rect
 * @return true if a good rectangle was found fully in the image
 */
bool
GetBlock(int x, int y, int width, int height, int cols, int rows, cv::Rect & rect)
{
  // If the box is not fully in the image, stop here
  if (x + width - 1 >= cols)
    return false;
  if (y + height - 1 >= rows)
    return false;

  rect.x = x;
  rect.y = y;
  // Try expanding the rectangle if a neighboring rectangle would be out of the image
  if (x + 2 * width - 1 >= cols)
    rect.width = cols - x;
  else
    rect.width = width;

  if (y + 2 * height - 1 >= rows)
    rect.height = rows - y;
  else
    rect.height = width;

  return true;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 *
 * @param point3d it is already a submatrix of the original points3d it is h x w x 3
 * @param r
 * @param p0
 * @return
 */
bool
IsBlockOnPlane(cv::Rect rect, const cv::Mat & points3d, const cv::Vec3f & r, const cv::Vec3f & p0, float err,
               cv::Mat_<uchar> & overall_mask, cv::Mat_<uchar> & mask, int index_plane)
{
  cv::Mat point3d_reshape;
  points3d(cv::Range(rect.y, rect.y + rect.height), cv::Range(rect.x, rect.x + rect.width)).copyTo(point3d_reshape);
  size_t n_points = point3d_reshape.cols * point3d_reshape.rows;
  // Make the matrx cols*ros x 3
  point3d_reshape = point3d_reshape.reshape(1, n_points);
  cv::Mat errs = point3d_reshape * (cv::Mat_<float>(3, 1) << r[0], r[1], r[2]);
  errs = errs - r.dot(p0);

  // Find the points on the plane
  cv::Mat good_points = cv::abs(errs) < err;
  good_points = good_points.reshape(1, rect.height);

  // Fill the mask with the valid points
  cv::Mat_<uchar> sub_mask = mask(cv::Range(rect.y, rect.y + rect.height), cv::Range(rect.x, rect.x + rect.width));
  sub_mask.setTo(cv::Scalar(index_plane), good_points);
  if (sub_mask(0, 0) != index_plane)
    sub_mask(0, 0) = 255 - index_plane;

  sub_mask = overall_mask(cv::Range(rect.y, rect.y + rect.height), cv::Range(rect.x, rect.x + rect.width));
  sub_mask.setTo(cv::Scalar(index_plane), good_points);

  return cv::countNonZero(cv::abs(errs) < err) > (n_points / 2);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
keep_in_boundaries(int cols, int rows, cv::Point2i &point)
{
  if (point.x < 0)
    point.x = 0;
  else if (point.x >= cols)
    point.x = cols - 1;
  if (point.y < 0)
    point.y = 0;
  else if (point.y >= rows)
    point.y = rows - 1;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace tabletop
{
  struct PlaneFinder
  {
    static void
    declare_params(tendrils& params)
    {
      params.declare(&PlaneFinder::n_max_, "n_max", "Maximum total number of 3d points.", 2000);
      params.declare(&PlaneFinder::k_max_, "k_max", "Maximum number of neighborhoods to sample.", 100);
      params.declare(&PlaneFinder::l_, "l", "Number of local samples.", 50);
      params.declare(&PlaneFinder::nu_, "nu", "Neighborhood for global samples (in pixel).", 30);
      params.declare(&PlaneFinder::S_, "S", "Plane size in world space for local samples.", 1);
      params.declare(&PlaneFinder::epsilon_, "epsilon", "Maximum plane offset error for inliers.", 0.02);
      params.declare(&PlaneFinder::alpha_in_, "alpha_in", "Minimum fraction of inliers to accept local sample.", 0.4);
    }

    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare(&PlaneFinder::image_, "image", "The current gray frame.").required(true);
      inputs.declare(&PlaneFinder::points3d_, "point3d", "The current depth frame.").required(true);
      inputs.declare(&PlaneFinder::K_, "K", "The camera intrinsic parameter matrix.").required(true);

      outputs.declare(&PlaneFinder::convex_hull_image_, "image",
                      "The depth image with the convex hulls for the planes.");
    }

    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      colors_.clear();
      colors_.push_back(cv::Scalar(255, 255, 0));
      colors_.push_back(cv::Scalar(0, 255, 255));
      colors_.push_back(cv::Scalar(255, 0, 255));
      colors_.push_back(cv::Scalar(255, 0, 0));
      colors_.push_back(cv::Scalar(0, 255, 0));
      colors_.push_back(cv::Scalar(0, 0, 255));
      colors_.push_back(cv::Scalar(0, 0, 0));
      colors_.push_back(cv::Scalar(85, 85, 85));
      colors_.push_back(cv::Scalar(170, 170, 170));
      colors_.push_back(cv::Scalar(255, 255, 255));
    }

    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      cv::Mat K;
      K_->convertTo(K, CV_32F);

      // Pre-computations
      const float inv_fx = 1.f / K.at<float>(0, 0);
      const float inv_fy = 1.f / K.at<float>(1, 1);
      const float ox = K.at<float>(0, 2);
      const float oy = K.at<float>(1, 2);
      float tan_fh = tan(2 * atan(points3d_->cols / (2 * K.at<float>(0, 0))) * 180.0 / CV_PI);
      float tan_fv = tan(2 * atan(points3d_->rows / (2 * K.at<float>(1, 1))) * 180.0 / CV_PI);
      cv::Mat convex_hull_image;
      image_->copyTo(convex_hull_image);
      cv::Mat_<uchar> overall_mask = cv::Mat_<uchar>::zeros(points3d_->rows, points3d_->cols);
      std::vector<cv::Mat_<uchar> > masks;

      // Line 2
      std::vector<cv::Point3f> P;
      std::vector<cv::Vec3f> R;
      std::vector<std::vector<cv::Point3f> > C;

      // Line 3-4
      size_t n = 0;
      size_t k = 0;
      size_t index_plane = 0;

      std::vector<cv::Point2i> d(3);
      std::vector<cv::Point3f> p(3);
      cv::Vec3f r;
      std::vector<cv::Point2i> D_hat;
      std::vector<cv::Point3f> P_hat;
      std::vector<cv::Vec3f> R_hat;
      std::vector<cv::Point3f> C_hat;
      P_hat.reserve(*l_);
      R_hat.reserve(*l_);
      C_hat.reserve(*l_);
      while ((n < *n_max_) && (k < *k_max_))
      {
        // Line 5-8
        ++k;
        d[0].x = rng_.uniform(0, points3d_->cols);
        d[0].y = rng_.uniform(0, points3d_->rows);
        if ((cvIsNaN(points3d_->at<float>(d[0].y, d[0].x, 0))) || overall_mask(d[0].y, d[0].x))
          continue;

        int threshold = (*nu_) * (*nu_);
        CreatePoint(d[0], *points3d_, overall_mask, *nu_, d[1]);
        if (PointDistanceSq(d[0], d[1]) <= threshold)
          continue;

        CreatePoint(d[0], *points3d_, overall_mask, *nu_, d[2]);
        if ((PointDistanceSq(d[2], d[0]) <= threshold) || (PointDistanceSq(d[2], d[1]) <= threshold))
          continue;

        // Line 9
        for (unsigned char i = 0; i < 3; ++i)
          p[i] = points3d_->at<cv::Point3f>(d[i].y, d[i].x);

        // Line 10
        r = (p[1] - p[0]).cross(p[2] - p[0]);
        r = r / cv::norm(r);

        // Line 11
        float z_mean = (p[0].z + p[1].z + p[2].z) / 3;

        // Line 12-13
        float w_prime = points3d_->cols * (*S_) / z_mean * tan_fh;
        float h_prime = points3d_->rows * (*S_) / z_mean * tan_fv;

        // Line 14-18
        size_t numInliers = 0;
        D_hat.clear();
        D_hat.push_back(d[0]);
        D_hat.push_back(d[1]);
        D_hat.push_back(d[2]);
        P_hat.clear();
        R_hat.clear();
        C_hat.clear();

        for (size_t j = 3; j < (*l_); ++j)
        {
          // Line 19
          cv::Point2i d_j;
          d_j.y = d[0].y + rng_.uniform(-h_prime / 2, h_prime / 2);
          d_j.x = d[0].x + rng_.uniform(-w_prime / 2, w_prime / 2);
          keep_in_boundaries(points3d_->cols, points3d_->rows, d_j);
          if (overall_mask(d_j.y, d_j.x))
            continue;

          // Line 20
          cv::Point3f p_j = points3d_->at<cv::Point3f>(d_j.y, d_j.x);
          if (cvIsNaN(p_j.x))
            continue;

          // Line 21-27
          float e = std::abs(r.dot(p_j - p[0]));
          if (e < *epsilon_)
          {
            D_hat.push_back(d_j);
            P_hat.push_back(p_j);
            R_hat.push_back(r);
            ++numInliers;
          }
        }
        // Line 28-30
        if (!(numInliers > (*alpha_in_) * (*l_)))
          continue;

        P.insert(P.end(), P_hat.begin(), P_hat.end());
        R.insert(R.end(), R_hat.begin(), R_hat.end());

        // Skip 31-33: we get a convex hull differently

        ++index_plane;
        cv::Mat_<uchar> mask = cv::Mat_<uchar>::zeros(points3d_->rows, points3d_->cols);
        masks.push_back(mask);

        // Process blocks starting at d[0]
        int block_size = 40;
        float err = 0.02;
        // Keep track of the blocks to process
        std::deque<cv::Rect> blocks;

        {
          // Find the first block to start from: it has to be fully in the image
          cv::Rect rect;
          int x = (d[0].x / block_size) * block_size;
          int y = (d[0].y / block_size) * block_size;
          if (!GetBlock(x, y, block_size, block_size, points3d_->cols, points3d_->rows, rect))
            if (!GetBlock(x - block_size, y, block_size, block_size, points3d_->cols, points3d_->rows, rect))
              if (!GetBlock(x, y - block_size, block_size, block_size, points3d_->cols, points3d_->rows, rect))
                GetBlock(x - block_size, y - block_size, block_size, block_size, points3d_->cols, points3d_->rows,
                         rect);
          blocks.push_back(rect);
        }

        while (!blocks.empty())
        {
          cv::Rect block = blocks.front();
          blocks.pop_front();

          // If the mask has already been processed, just skip right away
          if (mask(block.y, block.x))
            continue;

          // Don't look at the neighboring blocks if this is not a fitting block
          if (!IsBlockOnPlane(block, (*points3d_), r, p[0], err, overall_mask, mask, index_plane))
            continue;

          // Add neighboring blocks if they have not been processed
          for (int y = block.y - block_size; y <= block.y + block_size; y += block_size)
          {
            if ((y < 0) || (y >= points3d_->rows))
              continue;
            for (int x = block.x - block_size; x <= block.x + block_size; x += block_size)
            {
              if ((x < 0) || (x >= points3d_->cols))
                continue;
              // Do not process the block if it has been processed already
              if (mask(y, x))
                continue;
              cv::Rect new_block;
              // Make sure we can get a valid block
              if (!GetBlock(x, y, block_size, block_size, points3d_->cols, points3d_->rows, new_block))
                continue;
              blocks.push_back(new_block);
            }
          }
        }

        // Display the convex hull in the images
        //convex_hull_image_
        // Line 33
        n += numInliers;

        // Extra for display
        // Display the sampled points
        BOOST_FOREACH(cv::Point2i & point, D_hat)
        {
          point.x = (point.x * convex_hull_image.cols) / points3d_->cols;
          point.y = (point.y * convex_hull_image.rows) / points3d_->rows;
          cv::circle(convex_hull_image, point, 3, cv::Scalar(0, 255, 0), -1);
        }
      };

      //// Perform some display

      // Resize the current masks
      std::vector<cv::Mat> resized_masks(masks.size());
      for (size_t i = 0; i < masks.size(); ++i)
        cv::resize(masks[i], resized_masks[i], cv::Size(256, (masks[i].rows * 256) / masks[i].cols));

      // Compare each mask to the previous ones
      cv::Mat_<int> overlap(resized_masks.size(), previous_resized_masks_.size());
      for (size_t i = 0; i < resized_masks.size(); ++i)
        for (size_t j = 0; j < previous_resized_masks_.size(); ++j)
        {
          cv::Mat and_res;
          cv::bitwise_and(resized_masks[i], previous_resized_masks_[j], and_res);
          overlap(i, j) = cv::countNonZero(and_res);
        }
      std::cout << overlap << std::endl;

      // Maps a new index to the corresponding old index
      std::map<int, int> index_map;
      for (size_t i = 0; i < resized_masks.size(); ++i)
        index_map[i] = i;
      while (true)
      {
        // Find the best overlap
        int max_overlap = 0, max_i, max_j;
        for (size_t i = 0; i < resized_masks.size(); ++i)
          for (size_t j = 0; j < previous_resized_masks_.size(); ++j)
          {
            if (overlap(i, j) > max_overlap)
            {
              max_overlap = overlap(i, j);
              max_i = i;
              max_j = j;
            }
          }
        if (max_overlap == 0)
          break;
        index_map[max_i] = max_j;
        // Reset some overlap values
        for (size_t i = 0; i < overlap.rows; ++i)
          overlap(i, max_j) = 0;
        for (size_t j = 0; j < overlap.cols; ++j)
          overlap(max_i, j) = 0;
      }

      //// Draw the contours with the right color

      // Display the contours of the plane
      {
        size_t index_plane = 0;
        BOOST_FOREACH(const cv::Mat & mask, masks)
        {
          std::vector<std::vector<cv::Point2i> > contours;
          std::vector<cv::Vec4i> hierarchy;
          cv::findContours(mask, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
          BOOST_FOREACH(std::vector<cv::Point2i> & contour, contours)
          {
            BOOST_FOREACH(cv::Point2i & point, contour)
            {
              point.x = (point.x * convex_hull_image.cols) / points3d_->cols;
              point.y = (point.y * convex_hull_image.rows) / points3d_->rows;
            }
          }
          if (index_plane < colors_.size())
            cv::drawContours(convex_hull_image, contours, -1, colors_[index_map[index_plane]], 3);
          ++index_plane;
        }
      }

      *convex_hull_image_ = convex_hull_image;
      previous_resized_masks_ = resized_masks;

      return ecto::OK;
    }

    /** Parameters from the paper */
    ecto::spore<size_t> n_max_;
    ecto::spore<size_t> k_max_;
    ecto::spore<size_t> l_;
    /** Neighborhood for global samples: an int and not size_t for computations */
    ecto::spore<int> nu_;
    ecto::spore<size_t> S_;
    ecto::spore<float> epsilon_;
    ecto::spore<float> alpha_in_;

    ecto::spore<cv::Mat> image_;
    ecto::spore<cv::Mat> points3d_;
    ecto::spore<cv::Mat> K_;

    /** The output rotation matrix */
    ecto::spore<cv::Mat> R_;
    /** The output translation matrix */
    ecto::spore<cv::Mat> T_;

    /** Random number generator */
    cv::RNG rng_;

    /** */
    ecto::spore<cv::Mat> convex_hull_image_;

    std::vector<cv::Scalar> colors_;
    /** Store the previous resize masks for color consistency */
    std::vector<cv::Mat> previous_resized_masks_;
  }
  ;
}

ECTO_CELL(tabletop_table, tabletop::PlaneFinder, "PlaneFinder",
          "Uses the RGBDOdometry to figure out where the camera is.")
