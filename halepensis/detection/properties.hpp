#pragma once

#include "geometry.hpp"

class Properties {
public:
    Vector position;
    Quaternion rotation;
    Vector mass_center;
    
    Properties(Vector position,
               Quaternion rotation,
               Vector mass_center);
};

auto detect_properties(const std::shared_ptr<PointCloud>& cloud) -> Properties;
