#include "object.hpp"
#include "utils.hpp"

Object::Object(std::string pcd_file)
{
    PointCloud::Ptr cloud (new PointCloud);
    cloud = loadCloud(pcd_file);
    cloud = removeBackground(cloud);
    cloud = downsample(cloud);
    feature_cloud = FeatureCloud(cloud);
}

PointCloud::Ptr Object::getPointCloud() const {
    return feature_cloud.getPointCloud();
}