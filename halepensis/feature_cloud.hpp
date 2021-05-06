#pragma once

#include "types.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/segmentation/sac_segmentation.h>
#pragma clang diagnostic pop

class FeatureCloud
{
public:
    FeatureCloud() = default;
    FeatureCloud(PointCloud::Ptr cloud);
    PointCloud::Ptr getPointCloud () const;
    SurfaceNormals::Ptr getSurfaceNormals () const;
    LocalFeatures::Ptr getLocalFeatures () const;

private:
    PointCloud::Ptr cloud;
    SurfaceNormals::Ptr normals;
    LocalFeatures::Ptr features;
    SearchMethod::Ptr search_method;
    float normal_radius;
    float feature_radius;

    void computeSurfaceNormals();
    void computeLocalFeatures();
};
