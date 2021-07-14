#include "entity_component.hpp"
#include "entity_surface.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/common/io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/model_types.h>
#pragma clang diagnostic pop

entity_component::entity_component(component_type type,
                     std::shared_ptr<point_indices> indices,
                     std::shared_ptr<point_cloud> cloud,
                     std::shared_ptr<model_coefficients> coefficients):
    type(type), indices(indices), cloud(cloud), coefficients(coefficients)
{
    
}

auto component_type_to_sac_model(const component_type type) -> pcl::SacModel
{
    switch (type) {
        case component_type::cylinder:
            return pcl::SACMODEL_CYLINDER;
        case component_type::surface:
            return pcl::SACMODEL_PLANE;
    }
}

auto recognize_components(const component_type type, const entity_surface& surface) -> std::vector<const entity_component>
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::SACSegmentation<point> seg;
    seg.setOptimizeCoefficients(true);
    const auto model = component_type_to_sac_model(type);
    seg.setModelType(model);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);
    
    pcl::ExtractIndices<point> extract;
    auto source_cloud = std::make_shared<point_cloud>();
    auto component_cloud = std::make_shared<point_cloud>();
    auto tmp_cloud = std::make_shared<point_cloud>();

    pcl::copyPointCloud(*surface.cloud, *source_cloud);
    
    std::vector<const entity_component> components;
    
    // Max 10 parts
    for (int i = 0; i < 10; ++i)
    {
        seg.setInputCloud(source_cloud);
        seg.segment(*inliers, *coefficients);
        
        if (inliers->indices.size() == 0) break;
        
        extract.setInputCloud(source_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*component_cloud);
        
        extract.setNegative (true);
        extract.filter(*tmp_cloud);
        source_cloud.swap(tmp_cloud);
        
        components.push_back(entity_component(type, inliers, component_cloud, coefficients));
        
        if (source_cloud->size() == 0) break;
    }
    
    return components;
}
