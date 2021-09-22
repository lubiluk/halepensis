#include "ransac.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/common/io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/model_types.h>
#pragma clang diagnostic pop

auto fit_cylinder(const std::shared_ptr<point_cloud>& input_cloud,
                  const std::shared_ptr<surface_normals>& input_normals)
-> std::optional<std::tuple<std::shared_ptr<point_indices>, std::shared_ptr<model_coefficients>>>
{
    pcl::SACSegmentationFromNormals<point, normal> seg;
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.1);
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.05);
    seg.setRadiusLimits(0, 0.5);
    
    auto coefficients = std::make_shared<model_coefficients>();
    auto inliers = std::make_shared<point_indices>();
    
    seg.setInputCloud(input_cloud);
    seg.setInputNormals(input_normals);
    seg.segment(*inliers, *coefficients);
    
    // Finish early if can't find any more inliners.
    if (inliers->indices.size() == 0) return std::nullopt;
    
    return std::optional(std::make_tuple(inliers, coefficients));
}

auto fit_plane(const std::shared_ptr<point_cloud>& input_cloud)
-> std::optional<std::tuple<std::shared_ptr<point_indices>, std::shared_ptr<model_coefficients>>>
{
    pcl::SACSegmentation<point> seg;
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    
    auto coefficients = std::make_shared<model_coefficients>();
    auto inliers = std::make_shared<point_indices>();
    
    seg.setInputCloud(input_cloud);
    seg.segment(*inliers, *coefficients);
    
    // Finish early if can't find any more inliners.
    if (inliers->indices.size() == 0) return std::nullopt;
    
    return std::optional(std::make_tuple(inliers, coefficients));
}

auto fit_line(const std::shared_ptr<point_cloud>& input_cloud)
-> std::optional<std::tuple<std::shared_ptr<point_indices>, std::shared_ptr<model_coefficients>>>
{
    pcl::SACSegmentation<point> seg;
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    
    auto coefficients = std::make_shared<model_coefficients>();
    auto inliers = std::make_shared<point_indices>();
    
    seg.setInputCloud(input_cloud);
    seg.segment(*inliers, *coefficients);
    
    // Finish early if can't find any more inliners.
    if (inliers->indices.size() == 0) return std::nullopt;
    
    return std::optional(std::make_tuple(inliers, coefficients));
}


