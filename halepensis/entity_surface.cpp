#include "entity_surface.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#pragma clang diagnostic pop


std::shared_ptr<sufrace_normals> compute_surface_normals(const std::shared_ptr<point_cloud> &cloud,
                             const std::shared_ptr<search_method> &search_method,
                             float normal_radius)
{
    const auto normals = std::make_shared<sufrace_normals>();

    pcl::NormalEstimation<Point, Normal> norm_est;
    norm_est.setInputCloud(cloud);
    norm_est.setSearchMethod(search_method);
    norm_est.setRadiusSearch(normal_radius);
    norm_est.compute(*normals);
    
    return normals;
}

std::shared_ptr<local_features> compute_local_features(const std::shared_ptr<point_cloud> &cloud,
                          const std::shared_ptr<sufrace_normals> &normals,
                          const std::shared_ptr<search_method> &search_method,
                          float feature_radius)
{
    const auto features = std::make_shared<local_features> ();

    pcl::FPFHEstimation<Point, Normal, pcl::FPFHSignature33> fpfh_est;
    fpfh_est.setInputCloud(cloud);
    fpfh_est.setInputNormals(normals);
    fpfh_est.setSearchMethod(search_method);
    fpfh_est.setRadiusSearch(feature_radius);
    fpfh_est.compute(*features);
    
    return features;
}

entity_surface::entity_surface(std::shared_ptr<point_cloud> cloud) :
cloud(cloud),
method(std::make_shared<search_method>()),
normal_radius(0.02f),
feature_radius(0.02f),
normals(compute_surface_normals(cloud, method, normal_radius)),
features(compute_local_features(cloud, normals, method, feature_radius))
{
    
}
