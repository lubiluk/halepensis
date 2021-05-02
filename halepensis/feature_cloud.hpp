#pragma once

#include "types.hpp"

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