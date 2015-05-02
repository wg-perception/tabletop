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

// Author(s): Marius Muja and Matei Ciocarlie

#include "tabletop_object_detector/model_fitter.h"

#include <moveit/distance_field/propagation_distance_field.h>
#include "tabletop_object_detector/iterative_distance_fitter.h"

#include <boost/bind.hpp>

namespace tabletop_object_detector {

DistanceFieldFitter::DistanceFieldFitter() : distance_voxel_grid_(NULL) 
{
  truncate_value_ = 0.05;
  distance_field_resolution_ = 0.002;
}

DistanceFieldFitter::~DistanceFieldFitter() 
{
  delete distance_voxel_grid_;
}

void DistanceFieldFitter::initializeFromVector(const std::vector<cv::Point3f> &points)
{
  delete distance_voxel_grid_;
  distance_voxel_grid_ = NULL;

  if (points.size() == 0) {
    return;
  }

  cv::Point3f min = points[0], max = points[0];
  for (size_t i=0; i<points.size(); ++i)
  {
    if (min.x > points[i].x)
      min.x = points[i].x;
    if (max.x < points[i].x)
      max.x = points[i].x;

    if (min.y > points[i].y)
      min.y = points[i].y;
    if (max.y < points[i].y)
      max.y = points[i].y;

    if (min.z > points[i].z)
      min.z = points[i].z;
    if (max.z < points[i].z)
      max.z = points[i].z;
  }

  //the distance field is initialized as follows: match the size of the object, but add
  //padding equal to the truncate_value_ on each side. Resolution is constant regardless 
  //of the size of the object.

  //the only difference in along negative Z where we add only a very small padding, making
  //the assumptions that we are pretty sure where the table is (Z=0 by convention), all objects
  //have the origin on the bottom, and nothing is below the table
  //allow just two cells under the table, to deal with noise and such
  double table_padding = 2.5 * distance_field_resolution_;
  distance_voxel_grid_ = new distance_field::PropagationDistanceField(max.x-min.x + 2*truncate_value_,
								      max.y-min.y + 2*truncate_value_,
								      max.z-min.z + truncate_value_ + table_padding,
								      distance_field_resolution_, 
								      min.x - truncate_value_,
								      min.y - truncate_value_,
								      min.z - table_padding,
								      2 * truncate_value_ );
  distance_voxel_grid_->reset();
  EigenSTL::vector_Vector3d eigen_points(points.size());
  for(size_t i = 0; i < points.size(); ++i)
  {
    eigen_points[i][0] = points[i].x;
    eigen_points[i][1] = points[i].y;
    eigen_points[i][2] = points[i].z;
  }
  distance_voxel_grid_->addPointsToField(eigen_points);
}

double dist(const cv::Point3f &v0, const cv::Point3f &v1)
{
  return sqrt( (v1.x-v0.x)*(v1.x-v0.x) +
	       (v1.y-v0.y)*(v1.y-v0.y) +
	       (v1.z-v0.z)*(v1.z-v0.z) );
}

/*! Given a triangle defined by three vertices, returns a set of points obtained
  by sampling the surface of the triangle. Points are obtained by barycentric
  interpolation, with a guarantee that, along the interpolated directions, the
  distance between interpolated locations is not greater than min_res  (could be 
  smaller if one of the 0-1 and 0-2 edges of the triangle is significantly shorter 
  than the other).

  The vertices themselves are NOT returned in the set of points.
*/
std::vector<cv::Point3f> interpolateTriangle(cv::Point3f v0, cv::Point3f v1,
					   cv::Point3f v2, double min_res)
{
  std::vector<cv::Point3f> vectors;

  // Choose which corner should be the main one
  double d01 = dist(v0, v1);
  double d02 = dist(v0, v2);
  double d12 = dist(v1, v2);
  cv::Point3f vtmp;
  if ((d01 < d02) && (d01 < d12)) {
      vtmp = v0;
      v0 = v2;
      v2 = vtmp;
  }
  if ((d02 < d01) && (d02 < d12)) {
    vtmp = v0;
    v0 = v1;
    v1 = vtmp;
  }
  d01 = dist(v0, v1);
  d02 = dist(v0, v2);
  d12 = dist(v1, v2);

  //find out the interpolation resolution for the first coordinate
  //based on the size of the 0-1 and 0-2 edges
  double res_0 = min_res / std::max(d01, d02);

  //perform the first interpolation
  //we do not want the vertices themselves, so we don't start at 0 
  double t0=res_0;
  bool done = false;
  while (!done)
  {
    if (t0 >= 1.0)
    {
      t0 = 1.0;
      done = true;
    }
    //compute the resolution for the second interpolation
    cv::Point3f p1 = t0*v0 + (1-t0) * v1;
    cv::Point3f p2 = t0*v0 + (1-t0) * v2;
    double d12 = dist(p1, p2);
    double res_12 = min_res / d12;

    //perform the second interpolation
    double t12 = 0;
    bool done12 = false;
    while (!done12)
    {
      if (t12 >= 1.0)
      {
	t12 = 1.0;
	done12 = true;
      }
      //actual point insertion
      //do not insert the vertices themselves
      if (t0!=1.0 || (t12!=0.0 && t12!=1.0))
      {
	vectors.push_back( t12*p1 + (1.0 - t12)*p2 );
      }
      t12 += res_12;
    }
    
    t0 += res_0;
  }
  return vectors;
}

void ModelToCloudFitter::sampleMesh(const shape_msgs::Mesh &mesh,
				    std::vector<cv::Point3f> &btVectors,
				    double resolution)
{
  btVectors.reserve(mesh.vertices.size());
  //vertices themselves get inserted explicitly here. If we inserted them
  //as part of triangles, we would insert each vertex multiple times
  typedef std::vector<geometry_msgs::Point>::const_iterator I;
  for (I i=mesh.vertices.begin(); i!=mesh.vertices.end(); i++) 
  {
    btVectors.push_back(cv::Point3f(i->x,i->y,i->z));
  }
  
  //sample triangle surfaces at a specified min-resolution 
  //and insert the resulting points
  for (size_t i=0; i<mesh.triangles.size(); ++i)
  {
    cv::Point3f v0( mesh.vertices[ mesh.triangles[i].vertex_indices[0] ].x,
		  mesh.vertices[ mesh.triangles[i].vertex_indices[0] ].y,
		  mesh.vertices[ mesh.triangles[i].vertex_indices[0] ].z);
    cv::Point3f v1( mesh.vertices[ mesh.triangles[i].vertex_indices[1] ].x,
		  mesh.vertices[ mesh.triangles[i].vertex_indices[1] ].y,
		  mesh.vertices[ mesh.triangles[i].vertex_indices[1] ].z);
    cv::Point3f v2( mesh.vertices[ mesh.triangles[i].vertex_indices[2] ].x,
		  mesh.vertices[ mesh.triangles[i].vertex_indices[2] ].y,
		  mesh.vertices[ mesh.triangles[i].vertex_indices[2] ].z);
    std::vector<cv::Point3f> triangleVectors = interpolateTriangle(v0, v1, v2, resolution);
    btVectors.insert(btVectors.begin(), triangleVectors.begin(), triangleVectors.end());
  }
}


void DistanceFieldFitter::initializeFromMesh(const shape_msgs::Mesh &mesh)
{
  std::vector<cv::Point3f> btVectors;
  model_points_.reserve(mesh.vertices.size());
  typedef std::vector<geometry_msgs::Point>::const_iterator I;
  for (I i = mesh.vertices.begin(); i != mesh.vertices.end(); i++)
    model_points_.push_back(cv::Point3f(i->x,i->y,i->z));
  // 20mm resolution
  //sampleMesh(mesh, model_points_, 0.02 );

  //we use a slightly larger resolution than the distance field, in an attempt to bring
  //down pre-computation time
  sampleMesh(mesh, btVectors,  1.5 * distance_field_resolution_ ); 
  initializeFromVector(btVectors);
}


} //namespace
