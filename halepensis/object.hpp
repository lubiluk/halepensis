#pragma once

#include "types.hpp"

#include "feature_cloud.hpp"
#include "part.hpp"

#include <initializer_list>
#include <vector>

class Object
{
public:
    std::vector<const FeatureCloud> feature_clouds;
    std::vector<const Part> parts;

    Object(std::initializer_list<std::string> const &pcd_files);
    std::vector<PointCloud::Ptr> getPointClouds() const;
    std::vector<PointCloud::ConstPtr> getConstPointClouds() const;
    auto findParts() -> void;
};
