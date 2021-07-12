#pragma once

#include "types.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>
#pragma clang diagnostic pop

enum class PartType {
    surface, cylinder
};

class Part {
public:
    const PartType type;
    const pcl::PointIndices::ConstPtr indices;
    const PointCloud::ConstPtr cloud;
    const pcl::ModelCoefficients::Ptr coefficients;
    
    Part(PartType type, pcl::PointIndices::ConstPtr indices, PointCloud::ConstPtr cloud, pcl::ModelCoefficients::Ptr coefficients);
};
