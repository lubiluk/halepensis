#include "sac_part_recognition.hpp"

#include "types.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/common/io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/model_types.h>
#pragma clang diagnostic pop

SacPartRecognition::SacPartRecognition(const int model_type, const PartType part_type):
model_type(model_type), part_type(part_type)
{
    
}

SacPartRecognition::~SacPartRecognition()
{
    
}

auto SacPartRecognition::recognize(const FeatureCloud& feature_cloud) const -> std::vector<const Part>
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::SACSegmentation<Point> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(model_type);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);
    
    pcl::ExtractIndices<Point> extract;
    PointCloud::Ptr source_cloud(new PointCloud), part_cloud(new PointCloud), tmp_cloud(new PointCloud);
    
    pcl::copyPointCloud(*feature_cloud.getPointCloud(), *source_cloud);
    
    std::vector<const Part> parts;
    
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
    
    return parts;
}
