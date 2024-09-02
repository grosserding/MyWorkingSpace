#ifndef PCL_ICP_2D_OMP_IMPL_H_
#define PCL_ICP_2D_OMP_IMPL_H_
#include <cmath>
#include <pcl/registration/eigen.h>
#include <pcl/registration/boost.h>
 
//
template <typename PointSource, typename PointTarget, typename Scalar> double
pcl::IterativeClosestPointOMP<PointSource, PointTarget, Scalar>::getFitnessScore(
    const PointCloudSourceConstPtr& cloud, double max_range) {
    // Transform the input dataset using the final transformation
    PointCloudSource input_transformed;
    if (cloud == nullptr) {
        transformPointCloud(*input_, input_transformed, final_transformation_);
    }
    else {
        input_transformed = *cloud;
    }
 
    // For each point in the source dataset
    double fitness_score = 0.0;
    int nr = 0;
 
#pragma omp parallel for reduction(+:fitness_score,nr)
    for (int i = 0; i < input_transformed.points.size(); ++i) {
        std::vector<int> nn_indices(1);
        std::vector<float> nn_dists(1);
        // Find its nearest neighbor in the target
        tree_->nearestKSearch(input_transformed.points[i], 1, nn_indices, nn_dists);
 
        // Deal with occlusions (incomplete targets)
        if (nn_dists[0] <= max_range) {
            // Add to the fitness score
            fitness_score += nn_dists[0];
            nr++;
        }
    }
 
    if (nr > 0)
        return (fitness_score / nr);
    else
        return (std::numeric_limits<double>::max());
 
}
#endif    // PCL_ICP_2D_OMP_IMPL_H_