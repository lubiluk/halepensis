#include "alignment.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#pragma clang diagnostic pop

auto align(const std::shared_ptr<point_cloud> &cloud1, const std::shared_ptr<point_cloud> &cloud2)
-> std::pair<Eigen::Matrix<float, 4, 4>, std::shared_ptr<point_cloud>>
{
    pcl::IterativeClosestPoint<point, point> icp;
    icp.setInputSource(cloud1);
    icp.setInputTarget(cloud2);
    icp.setMaxCorrespondenceDistance (0.05);
    icp.setMaximumIterations (5000);
    icp.setTransformationEpsilon (1e-8);
    icp.setEuclideanFitnessEpsilon (0.01);
    
    const auto result = std::make_shared<point_cloud>();
    icp.align(*result);
    icp.getFitnessScore();
    return {icp.getFinalTransformation(), result};
}
