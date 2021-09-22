#include "outliers.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/point_types.h>
#include <pcl/filters/model_outlier_removal.h>
#pragma clang diagnostic pop


auto filter_outliers(const std::shared_ptr<point_cloud>& input_cloud) -> std::shared_ptr<point_cloud>
{
    const auto result_cloud = std::make_shared<point_cloud>();
    
    pcl::ModelCoefficients sphere_coeff;
    sphere_coeff.values.resize (4);
    sphere_coeff.values[0] = 0;
    sphere_coeff.values[1] = 0;
    sphere_coeff.values[2] = 0;
    sphere_coeff.values[3] = 1;
    
    pcl::ModelOutlierRemoval<point> sphere_filter;
    sphere_filter.setModelCoefficients (sphere_coeff);
    sphere_filter.setThreshold (0.01);
    sphere_filter.setModelType (pcl::SACMODEL_SPHERE);
    sphere_filter.setInputCloud (input_cloud);
    sphere_filter.filter (*result_cloud);
    
    return result_cloud;
}
