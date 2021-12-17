#pragma once

#include "geometry.hpp"
#include <string>
#include <memory>

using entity_id = std::string;

enum class entity_type {
    object,
    mass_center,
    hole,
    peg
};

class scene_entity {
public:
    entity_type type;
    entity_id id;
    vec3 position;
    rot_mat orientation;
    std::shared_ptr<point_cloud> cloud;
    point min_corner;
    point max_corner;
    
    scene_entity(entity_type type,
           entity_id id,
           vec3 position,
           rot_mat orientation = rot_mat{},
           std::shared_ptr<point_cloud> cloud = nullptr,
           point min_corner = {},
           point max_corner = {});
    
    scene_entity();
    scene_entity(const scene_entity& original) = default;
    
    auto transformed(const mat44& transform) const -> scene_entity;
};
