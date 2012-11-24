/*********************************************************************
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

// Author(s): Marius Muja and Matei Ciocarlie
#include <string>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud.h>

#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>

#include <household_objects_database_msgs/DatabaseModelPose.h>
#include <household_objects_database_msgs/DatabaseModelPoseList.h>
#include <household_objects_database_msgs/GetModelMesh.h>

#include "tabletop_object_detector/exhaustive_fit_detector.h"
#include "tabletop_object_detector/marker_generator.h"
#include "tabletop_object_detector/iterative_distance_fitter.h"

#include <tabletop/object/tabletop_object_detector.h>

namespace tabletop_object_detector
{
  TabletopObjectRecognizer::TabletopObjectRecognizer()
  {
    detector_ = ExhaustiveFitDetector<IterativeTranslationFitter>();

    //initialize operational flags
    fit_merge_threshold_ = 0.05;
  }

  void
  TabletopObjectRecognizer::clearObjects()
  {
    detector_.clearObjects();
  }

  void
  TabletopObjectRecognizer::addObject(int model_id, const shape_msgs::Mesh & mesh)
  {
    detector_.addObject(model_id, mesh);
  }

  double
  TabletopObjectRecognizer::fitDistance(const ModelFitInfo &m1, const ModelFitInfo &m2)
  {
    double dx = m1.getPose().position.x - m2.getPose().position.x;
    double dy = m1.getPose().position.y - m2.getPose().position.y;
    double d = dx * dx + dy * dy;
    return sqrt(d);
  }
} //namespace tabletop_object_detector
