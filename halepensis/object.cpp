#include "object.hpp"
#include "utils.hpp"

#include <algorithm>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#pragma clang diagnostic pop

Object::Object(std::initializer_list<std::string> const &pcd_files)
{
    for (auto &f : pcd_files) {
        PointCloud::Ptr cloud (new PointCloud);
        cloud = loadCloud(f);
        cloud = removeBackground(cloud);
        cloud = downsample(cloud);
        feature_clouds.push_back(FeatureCloud(cloud));
    }
}

std::vector<PointCloud::Ptr> Object::getPointClouds() const
{
    std::vector<PointCloud::Ptr> out(feature_clouds.size());
    std::transform(feature_clouds.begin(), feature_clouds.end(), out.begin(), [](auto const &f) {
        return f.getPointCloud();
    });
    
    return out;
}

auto Object::findParts() -> void
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::SACSegmentation<Point> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);
    
    pcl::ExtractIndices<Point> extract;
    PointCloud::Ptr source_cloud(new PointCloud), part_cloud(new PointCloud), tmp_cloud(new PointCloud);
    
    pcl::copyPointCloud(*feature_clouds[0].getPointCloud(), *source_cloud);
    
    // Max 10 parts
    for (int i = 0; i < 10; ++i)
    {
        seg.setInputCloud(source_cloud);
        seg.segment(*inliers, *coefficients);
        
        if (inliers->indices.size() == 0) break;
        
        extract.setInputCloud(source_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*part_cloud);
        
        extract.setNegative (true);
        extract.filter(*tmp_cloud);
        source_cloud.swap(tmp_cloud);
        
        parts.push_back(Part(PartType::surface, inliers, part_cloud));
        
        if (source_cloud->size() == 0) break;
    }
}
