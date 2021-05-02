#include "feature_cloud.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#pragma clang diagnostic pop

FeatureCloud::FeatureCloud(PointCloud::Ptr cloud) : search_method(new SearchMethod),
                                                    normal_radius(0.02f),
                                                    feature_radius(0.02f),
                                                    cloud(cloud)
{
    computeSurfaceNormals();
    computeLocalFeatures();
}

PointCloud::Ptr FeatureCloud::getPointCloud() const
{
    return cloud;
}

SurfaceNormals::Ptr FeatureCloud::getSurfaceNormals() const
{
    return normals;
}

LocalFeatures::Ptr FeatureCloud::getLocalFeatures() const
{
    return features;
}

void FeatureCloud::computeSurfaceNormals()
{
    normals = SurfaceNormals::Ptr(new SurfaceNormals);

    pcl::NormalEstimation<Point, Normal> norm_est;
    norm_est.setInputCloud(cloud);
    norm_est.setSearchMethod(search_method);
    norm_est.setRadiusSearch(normal_radius);
    norm_est.compute(*normals);
}

void FeatureCloud::computeLocalFeatures()
{
    features = LocalFeatures::Ptr(new LocalFeatures);

    pcl::FPFHEstimation<Point, Normal, pcl::FPFHSignature33> fpfh_est;
    fpfh_est.setInputCloud(cloud);
    fpfh_est.setInputNormals(normals);
    fpfh_est.setSearchMethod(search_method);
    fpfh_est.setRadiusSearch(feature_radius);
    fpfh_est.compute(*features);
}
