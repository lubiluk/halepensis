#pragma once

#include "types.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/PointIndices.h>
#pragma clang diagnostic pop

enum class PartType {
    surface
};

class Part {
public:
    const PartType type;
    const pcl::PointIndices::ConstPtr indices;
    const PointCloud::ConstPtr cloud;
    
    Part(PartType type, pcl::PointIndices::ConstPtr indices, PointCloud::ConstPtr cloud);
};
