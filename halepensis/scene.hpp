#pragma once

#include "types.hpp"
#include "feature_cloud.hpp"

class Object;

class Scene
{
public:
    FeatureCloud feature_cloud;

    Scene(std::string pcd_file);
    PointCloud::Ptr getPointCloud() const;
    PointCloud::Ptr findObject(const Object& object) const;
};
