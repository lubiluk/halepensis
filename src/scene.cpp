#include "scene.hpp"
#include "utils.hpp"
#include "template_alignment.hpp"
#include <pcl/point_cloud.h>

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

PointCloud::Ptr Scene::findObject(Object &object)
{
    TemplateAlignment template_align;
    template_align.addTemplateCloud(object.feature_cloud);
    template_align.setTargetCloud(feature_cloud);

    // Find the best template alignment
    TemplateAlignment::Result best_alignment;
    int best_index = template_align.findBestAlignment(best_alignment);
    // const FeatureCloud &best_template = object_templates[best_index];

    // Save the aligned template for visualization
    PointCloud::Ptr transformed_cloud(new PointCloud);
    pcl::transformPointCloud(*(object.feature_cloud.getPointCloud()), *transformed_cloud, best_alignment.final_transformation);

    return transformed_cloud;
}
