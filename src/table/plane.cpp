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

#include <numeric>
#include <string>

#include <boost/foreach.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/contrib/contrib.hpp>

#include <ecto/ecto.hpp>

using ecto::tendrils;

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

      // Line 2
      std::vector<cv::Point3f> P;
      std::vector<cv::Vec3f> R;
      std::vector<std::vector<cv::Point3f> > C;
      std::vector<cv::Point3f> O;

      // Line 3-4
      size_t n = 0;
      size_t k = 0;

      std::vector<cv::Point2i> d;
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
        d.resize(3);
        d[0].x = rng_.uniform(*nu_, points3d_->cols - (*nu_));
        d[0].y = rng_.uniform(*nu_, points3d_->rows - (*nu_));
        if (cvIsNaN(points3d_->at<float>(d[0].y, d[0].x, 0)))
          continue;
        d[1].x = d[0].x + rng_.uniform(-(*nu_), (*nu_) + 1);
        d[1].y = d[0].y + rng_.uniform(-(*nu_), (*nu_) + 1);
        if (cvIsNaN(points3d_->at<float>(d[1].y, d[1].x, 0)))
          continue;
        d[2].x = d[0].x + rng_.uniform(-(*nu_), (*nu_) + 1);
        d[2].y = d[0].y + rng_.uniform(-(*nu_), (*nu_) + 1);
        if (cvIsNaN(points3d_->at<float>(d[2].y, d[2].x, 0)))
          continue;

        // Line 9
        for (unsigned char i = 0; i < 3; ++i)
          p[i] = points3d_->at < cv::Point3f > (d[i].y, d[i].x);

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

          // Line 20
          cv::Point3f p_j = points3d_->at < cv::Point3f > (d_j.y, d_j.x);

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
        if (numInliers > (*alpha_in_) * (*l_))
        {
          P.insert(P.end(), P_hat.begin(), P_hat.end());
          R.insert(R.end(), R_hat.begin(), R_hat.end());
          // Skip 31-33
          // Display the convex hull in the images
          //convex_hull_image_
          // Line 33
          n += numInliers;

          // Extra for display
          BOOST_FOREACH(cv::Point2i & point, D_hat)
          {
            point.x = (point.x * convex_hull_image.cols) / points3d_->cols;
            point.y = (point.y * convex_hull_image.rows) / points3d_->rows;
          }
          std::vector<cv::Point2i> hull;
          cv::convexHull(D_hat, hull);

          //cv::drawContours(convex_hull_image, std::vector<std::vector<cv::Point2i> >(1, hull), 0,
          //             cv::Scalar(255, 0, 0));
          BOOST_FOREACH(cv::Point2i & point, D_hat)
            cv::circle(convex_hull_image, point, 3, cv::Scalar(0, 255, 0), -1);
        }
        else
        {
          O.insert(O.end(), P_hat.begin(), P_hat.end());
        }
      };

      *convex_hull_image_ = convex_hull_image;

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
  };
}

ECTO_CELL(tabletop_table, tabletop::PlaneFinder, "PlaneFinder",
          "Uses the RGBDOdometry to figure out where the camera is.")
