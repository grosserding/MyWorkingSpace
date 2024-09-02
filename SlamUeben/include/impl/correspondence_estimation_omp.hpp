/* * Software License Agreement (BSD License)
 * *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 * *  All rights reserved.
 * *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 * *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
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
 * * $Id$
 * */

#ifndef PCL_REGISTRATION_IMPL_CORRESPONDENCE_ESTIMATION_OMP_H_
#define PCL_REGISTRATION_IMPL_CORRESPONDENCE_ESTIMATION_OMP_H_

#include <pcl/common/copy_point.h>
#include <pcl/common/io.h>

namespace pcl {

namespace registration {

template <typename PointSource, typename PointTarget, typename Scalar>
void CorrespondenceEstimationOMPT<PointSource, PointTarget, Scalar>::
    determineCorrespondences(pcl::Correspondences& correspondences,
                             double max_distance) {
  if (!initCompute()) return;

  double max_dist_sqr = max_distance * max_distance;
  unsigned int nr_valid_correspondences = 0;
  pcl::Correspondences temp_correspondences;
  temp_correspondences.resize(indices_->size());

  // Check if the template types are the same. If true, avoid a copy.
  // Both point types MUST be registered using the
  // POINT_CLOUD_REGISTER_POINT_STRUCT macro!
  if (isSamePointType<PointSource, PointTarget>()) {
    // Iterate over the input set of source indices
#pragma omp parallel for reduction(+ : nr_valid_correspondences)
    for (int i = 0; i < indices_->size(); ++i) {
      int idx = (*indices_)[i];
      std::vector<int> index(1);
      std::vector<float> distance(1);
      tree_->nearestKSearch(input_->points[idx], 1, index, distance);
      if (distance[0] > max_dist_sqr) {
        continue;
      }

      temp_correspondences[i].index_query = idx;
      temp_correspondences[i].index_match = index[0];
      temp_correspondences[i].distance = distance[0];
      nr_valid_correspondences++;
    }

  } else {
    // Iterate over the input set of source indices
#pragma omp parallel for reduction(+ : nr_valid_correspondences)
    for (int i = 0; i < indices_->size(); ++i) {
      int idx = (*indices_)[i];
      // Copy the source data to a target PointTarget format so we can search in
      // the tree
      PointTarget pt;
      copyPoint(input_->points[idx], pt);
      std::vector<int> index(1);
      std::vector<float> distance(1);
      tree_->nearestKSearch(pt, 1, index, distance);
      if (distance[0] > max_dist_sqr) {
        continue;
      }

      temp_correspondences[i].index_query = idx;
      temp_correspondences[i].index_match = index[0];
      temp_correspondences[i].distance = distance[0];
      nr_valid_correspondences++;
    }
  }
  correspondences.clear();
  correspondences.reserve(nr_valid_correspondences);
  for (const auto& corr : temp_correspondences) {
    if (corr.index_match < 0) {
      continue;
    }
    correspondences.push_back(corr);
  }
  deinitCompute();
}

///
template <typename PointSource, typename PointTarget, typename Scalar>
void CorrespondenceEstimationOMPT<PointSource, PointTarget, Scalar>::
    determineReciprocalCorrespondences(pcl::Correspondences& correspondences,
                                       double max_distance) {
  if (!initCompute()) return;

  // setup tree for reciprocal search
  // Set the internal point representation of choice
  if (!initComputeReciprocal()) return;
  double max_dist_sqr = max_distance * max_distance;

  pcl::Correspondences temp_correspondences;
  temp_correspondences.resize(indices_->size());

  unsigned int nr_valid_correspondences = 0;

  // Check if the template types are the same. If true, avoid a copy.
  // Both point types MUST be registered using the
  // POINT_CLOUD_REGISTER_POINT_STRUCT macro!
  if (isSamePointType<PointSource, PointTarget>()) {
    // Iterate over the input set of source indices
#pragma omp parallel for reduction(+ : nr_valid_correspondences)
    for (int i = 0; i < indices_->size(); ++i) {
      int idx = (*indices_)[i];
      std::vector<int> index(1);
      std::vector<float> distance(1);

      tree_->nearestKSearch(input_->points[idx], 1, index, distance);
      if (distance[0] > max_dist_sqr) {
        continue;
      }

      int target_idx = index[0];
      std::vector<int> index_reciprocal(1);
      std::vector<float> distance_reciprocal(1);
      tree_reciprocal_->nearestKSearch(target_->points[target_idx], 1,
                                       index_reciprocal, distance_reciprocal);
      if (distance_reciprocal[0] > max_dist_sqr || idx != index_reciprocal[0]) {
        continue;
      }

      temp_correspondences[i].index_query = idx;
      temp_correspondences[i].index_match = index[0];
      temp_correspondences[i].distance = distance[0];
    }
  } else {
    // Iterate over the input set of source indices
#pragma omp parallel for reduction(+ : nr_valid_correspondences)
    for (int i = 0; i < indices_->size(); ++i) {
      int idx = (*indices_)[i];
      std::vector<int> index(1);
      std::vector<float> distance(1);

      // Copy the source data to a target PointTarget format so we can search in
      // the tree
      PointTarget pt_src;
      copyPoint(input_->points[idx], pt_src);

      tree_->nearestKSearch(pt_src, 1, index, distance);
      if (distance[0] > max_dist_sqr) continue;

      int target_idx = index[0];
      std::vector<int> index_reciprocal(1);
      std::vector<float> distance_reciprocal(1);
      // Copy the target data to a target PointSource format so we can search in
      // the tree_reciprocal
      PointSource pt_tgt;
      copyPoint(target_->points[target_idx], pt_tgt);

      tree_reciprocal_->nearestKSearch(pt_tgt, 1, index_reciprocal,
                                       distance_reciprocal);
      if (distance_reciprocal[0] > max_dist_sqr || idx != index_reciprocal[0]) {
        continue;
      }

      temp_correspondences[i].index_query = idx;
      temp_correspondences[i].index_match = index[0];
      temp_correspondences[i].distance = distance[0];
    }
  }
  correspondences.clear();
  correspondences.reserve(nr_valid_correspondences);
  for (const auto& corr : temp_correspondences) {
    if (corr.index_match < 0) {
      continue;
    }
    correspondences.push_back(corr);
  }

  deinitCompute();
}

}  // namespace registration
}  // namespace pcl

//#define PCL_INSTANTIATE_CorrespondenceEstimation(T,U) template class
//PCL_EXPORTS
// pcl::registration::CorrespondenceEstimation<T,U>;

#endif /* PCL_REGISTRATION_IMPL_CORRESPONDENCE_ESTIMATION_H_ */