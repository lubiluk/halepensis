#pragma once

#include "geometry.hpp"


class Properties {
public:
    vec3 position;
    rot_mat rotation;
    vec3 mass_center;
    point min_corner;
    point max_corner;
    
    Properties(vec3 position,
               rot_mat rotation,
               vec3 mass_center,
               point min_corner,
               point max_corner);
};

auto detect_properties(const std::shared_ptr<point_cloud>& cloud) -> Properties;
