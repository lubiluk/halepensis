#include "scene.hpp"
#include "utils.hpp"
#include "template_alignment.hpp"
#include "object.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/point_cloud.h>
#pragma clang diagnostic pop

Scene::Scene(std::string pcd_file)
{
    PointCloud::Ptr cloud(new PointCloud);
    cloud = loadCloud(pcd_file);
    cloud = removeBackground(cloud);
    cloud = downsample(cloud);
    feature_cloud = FeatureCloud(cloud);
}

PointCloud::Ptr Scene::getPointCloud() const
{
    return feature_cloud.getPointCloud();
}

PointCloud::Ptr Scene::findObject(const Object &object) const
{
    TemplateAlignment template_align;
    
    for (const auto &fc : object.feature_clouds)
    {
        template_align.addTemplateCloud(fc);
    }
    
    template_align.setTargetCloud(feature_cloud);

    // Find the best template alignment
    TemplateAlignment::Result best_alignment;
    const int best_index = template_align.findBestAlignment(best_alignment);
    const FeatureCloud &best_template = object.feature_clouds[best_index];
    
    // Save the aligned template for visualization
    PointCloud::Ptr transformed_cloud(new PointCloud);
    pcl::transformPointCloud(*best_template.getPointCloud(), *transformed_cloud, best_alignment.final_transformation);

    return transformed_cloud;
}
