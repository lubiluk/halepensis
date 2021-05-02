#pragma once

#include "types.hpp"
#include "object.hpp"

class Scene
{
public:
    FeatureCloud feature_cloud;

    Scene(std::string pcd_file);
    PointCloud::Ptr getPointCloud() const;
    PointCloud::Ptr findObject(Object &object);
};