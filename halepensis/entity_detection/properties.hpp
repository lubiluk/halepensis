#pragma once

#include "geometry.hpp"


class properties {
public:
    vec3 position;
    rot_mat rotation;
    vec3 mass_center;
    point min_corner;
    point max_corner;
    
    properties(vec3 position,
               rot_mat rotation,
               vec3 mass_center,
               point min_corner,
               point max_corner);
    
    auto transformed(const mat44& transform) const -> properties;
};

auto detect_properties(const std::shared_ptr<point_cloud>& cloud) -> properties;
