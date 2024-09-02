#ifndef PCL_ICP_OMP_H_
#define PCL_ICP_OMP_H_
 
// PCL includes
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/registration/default_convergence_criteria.h>
#include <pcl/registration/icp.h>
#include "correspondence_estimation_omp.h"
namespace pcl
{
  /** \brief @b IterativeClosestPoint provides a base implementation of the Iterative Closest Point algorithm. 
    * The transformation is estimated based on Singular Value Decomposition (SVD).
    *
    * The algorithm has several termination criteria:
    *
    * <ol>
    * <li>Number of iterations has reached the maximum user imposed number of iterations (via \ref setMaximumIterations)</li>
    * <li>The epsilon (difference) between the previous transformation and the current estimated transformation is smaller than an user imposed value (via \ref setTransformationEpsilon)</li>
    * <li>The sum of Euclidean squared errors is smaller than a user defined threshold (via \ref setEuclideanFitnessEpsilon)</li>
    * </ol>
    *
    *
    * Usage example:
    * \code
    * IterativeClosestPoint<PointXYZ, PointXYZ> icp;
    * // Set the input source and target
    * icp.setInputCloud (cloud_source);
    * icp.setInputTarget (cloud_target);
    *
    * // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    * icp.setMaxCorrespondenceDistance (0.05);
    * // Set the maximum number of iterations (criterion 1)
    * icp.setMaximumIterations (50);
    * // Set the transformation epsilon (criterion 2)
    * icp.setTransformationEpsilon (1e-8);
    * // Set the euclidean distance difference epsilon (criterion 3)
    * icp.setEuclideanFitnessEpsilon (1);
    *
    * // Perform the alignment
    * icp.align (cloud_source_registered);
    *
    * // Obtain the transformation that aligned cloud_source to cloud_source_registered
    * Eigen::Matrix4f transformation = icp.getFinalTransformation ();
    * \endcode
    *
    * \author Radu B. Rusu, Michael Dixon
    * \ingroup registration
    */
  template <typename PointSource, typename PointTarget, typename Scalar = float>
  class IterativeClosestPointOMP : public IterativeClosestPoint<PointSource, PointTarget, Scalar>
  {
  public:
      typedef typename Registration<PointSource, PointTarget, Scalar>::PointCloudSource PointCloudSource;
      typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
      typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;
 
      typedef typename Registration<PointSource, PointTarget, Scalar>::PointCloudTarget PointCloudTarget;
      typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
      typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;
 
      typedef PointIndices::Ptr PointIndicesPtr;
      typedef PointIndices::ConstPtr PointIndicesConstPtr;
 
      typedef boost::shared_ptr<IterativeClosestPointOMP<PointSource, PointTarget, Scalar> > Ptr;
      typedef boost::shared_ptr<const IterativeClosestPointOMP<PointSource, PointTarget, Scalar> > ConstPtr;
      /** \brief Empty constructor. */
      IterativeClosestPointOMP()
          : IterativeClosestPoint<PointSource, PointTarget, Scalar>()
      {
          reg_name_ = "IterativeClosestPointOMP";
          transformation_estimation_.reset(new pcl::registration::TransformationEstimationSVD<PointSource, PointTarget, Scalar>());
          correspondence_estimation_.reset(new pcl::registration::CorrespondenceEstimationOMP<PointSource, PointTarget, Scalar>);
          convergence_criteria_.reset(new pcl::registration::DefaultConvergenceCriteria<Scalar>(nr_iterations_, transformation_, *correspondences_));
      };
 
      /** \brief Empty destructor */
      virtual ~IterativeClosestPointOMP() {}
 
      double getFitnessScore(const PointCloudSourceConstPtr& cloud,
          double max_range = std::numeric_limits<double>::max());
 
  };
}
 
 
#include "impl/icp_omp.hpp"
#endif  //#ifndef PCL_ICP_2D_OMP_H_