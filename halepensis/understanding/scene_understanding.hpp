#pragma once

#include "geometry.hpp"
#include "entity.hpp"
#include "relation.hpp"
#include <string>
#include <vector>
#include <memory>
#include "scene_graph.hpp"

class SceneUnderstanding {
public:
    const std::shared_ptr<PointCloud> cloud;
    SceneGraph graph;
    
    SceneUnderstanding(std::shared_ptr<PointCloud> cloud);
    auto object_clouds() const -> std::vector<std::shared_ptr<PointCloud>> ;
    auto objects() const -> std::vector<Entity>;
    auto features(const Entity& object) const -> std::vector<Entity>;
};
