#pragma once

#include "geometry.hpp"


class Properties {
public:
    Vector position;
    Rotation rotation;
    Vector mass_center;
    Point min_corner;
    Point max_corner;
    
    Properties(Vector position,
               Rotation rotation,
               Vector mass_center,
               Point min_corner,
               Point max_corner);
};

auto detect_properties(const std::shared_ptr<PointCloud>& cloud) -> Properties;
