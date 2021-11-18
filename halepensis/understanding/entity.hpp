#pragma once

#include "geometry.hpp"
#include <string>
#include <memory>

using EntityId = std::string;

class Entity {
public:
    enum class Type {
        object,
        mass_center,
        hole,
        peg
    };
    
    Type type;
    EntityId id;
    Vector position;
    Rotation orientation;
    std::shared_ptr<PointCloud> cloud;
    Point min_corner;
    Point max_corner;
    
    Entity(Type type,
           EntityId id,
           Vector position,
           Rotation orientation = Rotation{},
           std::shared_ptr<PointCloud> cloud = nullptr,
           Point min_corner = {},
           Point max_corner = {});
    
    Entity();
    Entity(const Entity& original) = default;
    
    auto transformed(const Transform& transform) const -> Entity;
};
