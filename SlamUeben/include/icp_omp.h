/* * Software License Agreement (BSD License)
 * *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
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

// PCL includes
// #include <pcl/memory.h>  // for dynamic_pointer_cast, pcl::make_shared, shared_ptr
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/default_convergence_criteria.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_symmetric_point_to_plane_lls.h>

#include "correspondence_estimation_omp.h"
namespace pcl {

template <typename PointSource, typename PointTarget, typename Scalar = float>
class IterativeClosestPointOMP
    : public IterativeClosestPoint<PointSource, PointTarget, Scalar> {
 public:
  typedef
      typename Registration<PointSource, PointTarget, Scalar>::PointCloudSource
          PointCloudSource;
  typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
  typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

  typedef
      typename Registration<PointSource, PointTarget, Scalar>::PointCloudTarget
          PointCloudTarget;
  typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
  typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

  typedef PointIndices::Ptr PointIndicesPtr;
  typedef PointIndices::ConstPtr PointIndicesConstPtr;

  typedef boost::shared_ptr<
      IterativeClosestPointOMP<PointSource, PointTarget, Scalar> >
      Ptr;
  typedef boost::shared_ptr<
      const IterativeClosestPointOMP<PointSource, PointTarget, Scalar> >
      ConstPtr;
  /** \brief Empty constructor. */
  IterativeClosestPointOMP() {
    this->reg_name_ = "IterativeClosestPointOMP";

    this->transformation_estimation_.reset(
        new pcl::registration::TransformationEstimationSVD<
            PointSource, PointTarget, Scalar>());
    this->correspondence_estimation_.reset(
        new pcl::registration::CorrespondenceEstimationOMPT<
            PointSource, PointTarget, Scalar>);
    this->convergence_criteria_.reset(
        new pcl::registration::DefaultConvergenceCriteria<Scalar>(
            this->nr_iterations_, this->transformation_,
            *this->correspondences_));

    /*
       transformation_estimation_.reset(new
       pcl::registration::TransformationEstimationSVD<PointSource, PointTarget,
       Scalar>()); correspondence_estimation_.reset(new
       pcl::registration::CorrespondenceEstimationOMP<PointSource, PointTarget,
       Scalar>); convergence_criteria_.reset(new
       pcl::registration::DefaultConvergenceCriteria<Scalar>(nr_iterations_,
       transformation_, *correspondences_));*/
  };

  /** \brief Empty destructor */
  virtual ~IterativeClosestPointOMP() {}

  // double getFitnessScore(const PointCloudSourceConstPtr& cloud,
  //    double max_range = std::numeric_limits<double>::max());
};

}  // namespace pcl