#pragma once

#include "types.hpp"
#include "feature_cloud.hpp"

class Object
{
public:
    FeatureCloud feature_cloud;

    Object(std::string pcd_file);
    PointCloud::Ptr getPointCloud() const;
};