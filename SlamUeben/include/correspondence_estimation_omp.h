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

#pragma once

#include <pcl/common/io.h>  // for getFields
// #include <pcl/memory.h>
#include <pcl/pcl_base.h>
#include <pcl/pcl_macros.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_types.h>
#include <pcl/search/kdtree.h>

#include <string>

namespace pcl {
namespace registration {

/** \brief @b CorrespondenceEstimation represents the base class for
 * determining correspondences between target and query point
 * sets/features.
 * * Code example:
 * * \code
 * pcl::PointCloud<pcl::PointXYZRGBA>::Ptr source, target;
 * // ... read or fill in source and target
 * pcl::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;
 * est.setInputSource (source);
 * est.setInputTarget (target);
 * * pcl::Correspondences all_correspondences;
 * // Determine all reciprocal correspondences
 * est.determineReciprocalCorrespondences (all_correspondences);
 * \endcode
 * * \author Radu B. Rusu, Michael Dixon, Dirk Holz
 * \ingroup registration
 */

template <typename PointSource, typename PointTarget, typename Scalar = float>
class CorrespondenceEstimationOMPT
    : public CorrespondenceEstimationBase<PointSource, PointTarget, Scalar> {
 public:
  using Ptr = shared_ptr<
      CorrespondenceEstimationOMPT<PointSource, PointTarget, Scalar>>;
  using ConstPtr = shared_ptr<
      const CorrespondenceEstimationOMPT<PointSource, PointTarget, Scalar>>;

  using CorrespondenceEstimationBase<PointSource, PointTarget,
                                     Scalar>::point_representation_;
  using CorrespondenceEstimationBase<PointSource, PointTarget,
                                     Scalar>::input_transformed_;
  using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::tree_;
  using CorrespondenceEstimationBase<PointSource, PointTarget,
                                     Scalar>::tree_reciprocal_;
  using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::target_;
  using CorrespondenceEstimationBase<PointSource, PointTarget,
                                     Scalar>::corr_name_;
  using CorrespondenceEstimationBase<PointSource, PointTarget,
                                     Scalar>::target_indices_;
  using CorrespondenceEstimationBase<PointSource, PointTarget,
                                     Scalar>::getClassName;
  using CorrespondenceEstimationBase<PointSource, PointTarget,
                                     Scalar>::initCompute;
  using CorrespondenceEstimationBase<PointSource, PointTarget,
                                     Scalar>::initComputeReciprocal;
  using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::input_;
  using CorrespondenceEstimationBase<PointSource, PointTarget,
                                     Scalar>::indices_;
  using CorrespondenceEstimationBase<PointSource, PointTarget,
                                     Scalar>::input_fields_;
  using PCLBase<PointSource>::deinitCompute;

  using KdTree = pcl::search::KdTree<PointTarget>;
  using KdTreePtr = typename KdTree::Ptr;

  using PointCloudSource = pcl::PointCloud<PointSource>;
  using PointCloudSourcePtr = typename PointCloudSource::Ptr;
  using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;

  using PointCloudTarget = pcl::PointCloud<PointTarget>;
  using PointCloudTargetPtr = typename PointCloudTarget::Ptr;
  using PointCloudTargetConstPtr = typename PointCloudTarget::ConstPtr;

  using PointRepresentationConstPtr =
      typename KdTree::PointRepresentationConstPtr;

  /** \brief Empty constructor. */
  CorrespondenceEstimationOMPT() { corr_name_ = "CorrespondenceEstimationOMP"; }

  /** \brief Empty destructor */
  ~CorrespondenceEstimationOMPT() override = default;

  /** \brief Determine the correspondences between input and target cloud.
   * \param[out] correspondences the found correspondences (index of query
   * point, index of target point, distance) \param[in] max_distance maximum
   * allowed distance between correspondences
   */
  void determineCorrespondences(
      pcl::Correspondences& correspondences,
      double max_distance = std::numeric_limits<double>::max()) override;

  /** \brief Determine the reciprocal correspondences between input and target
   * cloud. A correspondence is considered reciprocal if both Src_i has Tgt_i as
   * a correspondence, and Tgt_i has Src_i as one.
   * * \param[out] correspondences the found correspondences (index of query and
   * target point, distance) \param[in] max_distance maximum allowed distance
   * between correspondences
   */
  void determineReciprocalCorrespondences(
      pcl::Correspondences& correspondences,
      double max_distance = std::numeric_limits<double>::max()) override;

  /** \brief Clone and cast to CorrespondenceEstimationBase */
  typename CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::Ptr
  clone() const override {
    Ptr copy(new CorrespondenceEstimationOMPT<PointSource, PointTarget, Scalar>(
        *this));
    return (copy);
  }
};

}  // namespace registration
}  // namespace pcl

#include "impl/correspondence_estimation_omp.hpp"