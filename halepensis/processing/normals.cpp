#include "normals.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/search/pcl_search.h>
#include <pcl/kdtree/kdtree_flann.h>
#pragma clang diagnostic pop

auto compute_normals(const std::shared_ptr<point_cloud> &cloud,
                     const float normal_radius) -> std::shared_ptr<surface_normals>
{
    const auto normals = std::make_shared<surface_normals>();
    const auto search_method = std::make_shared<pcl::search::KdTree<point>>();
    
    pcl::NormalEstimation<point, normal> norm_est;
    norm_est.setInputCloud(cloud);
    norm_est.setSearchMethod(search_method);
    norm_est.setRadiusSearch(normal_radius);
    norm_est.compute(*normals);
    
    return normals;
}
