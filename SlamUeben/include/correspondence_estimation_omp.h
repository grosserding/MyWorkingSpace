#ifndef PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_OMP_H_
#define PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_OMP_H_
 
#include <string>
 
#include <pcl/pcl_base.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/pcl_macros.h>
 
#include <pcl/registration/correspondence_estimation.h>
 
namespace pcl
{
  namespace registration
  {
 
    /** \brief @b CorrespondenceEstimation represents the base class for
      * determining correspondences between target and query point
      * sets/features.
      *
      * Code example:
      *
      * \code
      * pcl::PointCloud<pcl::PointXYZRGBA>::Ptr source, target;
      * // ... read or fill in source and target
      * pcl::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;
      * est.setInputSource (source);
      * est.setInputTarget (target);
      *
      * pcl::Correspondences all_correspondences;
      * // Determine all reciprocal correspondences
      * est.determineReciprocalCorrespondences (all_correspondences);
      * \endcode
      *
      * \author Radu B. Rusu, Michael Dixon, Dirk Holz
      * \ingroup registration
      */
    template <typename PointSource, typename PointTarget, typename Scalar = float>
    class CorrespondenceEstimationOMP : public CorrespondenceEstimation<PointSource, PointTarget, Scalar>{
      public:
       /** \brief Empty constructor. */
       CorrespondenceEstimationOMP() {
         corr_name_ = "CorrespondenceEstimationOMP";
        }
      
        /** \brief Empty destructor */
        virtual ~CorrespondenceEstimationOMP() {}
 
        /** \brief Determine the correspondences between input and target cloud.
          * \param[out] correspondences the found correspondences (index of query point, index of target point, distance)
          * \param[in] max_distance maximum allowed distance between correspondences
          */
        virtual void 
        determineCorrespondences (pcl::Correspondences &correspondences,
                                  double max_distance = std::numeric_limits<double>::max ()) override;
 
        /** \brief Determine the reciprocal correspondences between input and target cloud.
          * A correspondence is considered reciprocal if both Src_i has Tgt_i as a 
          * correspondence, and Tgt_i has Src_i as one.
          *
          * \param[out] correspondences the found correspondences (index of query and target point, distance)
          * \param[in] max_distance maximum allowed distance between correspondences
          */
        virtual void 
        determineReciprocalCorrespondences (pcl::Correspondences &correspondences,
                                            double max_distance = std::numeric_limits<double>::max ()) override;
 
        
        /** \brief Clone and cast to CorrespondenceEstimationBase */
        virtual boost::shared_ptr< CorrespondenceEstimationBase<PointSource, PointTarget, Scalar> > 
        clone () const
        {
          Ptr copy (new CorrespondenceEstimationOMP<PointSource, PointTarget, Scalar> (*this));
          return (copy);
        }
     };
  }
}
 
#include "impl/correspondence_estimation_omp.hpp"
 
#endif /* PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_OMP_H_ */