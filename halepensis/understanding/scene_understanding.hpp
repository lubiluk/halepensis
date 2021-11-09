#pragma once

#include "geometry.hpp"
#include "scene_object.hpp"
#include <string>
#include <vector>

class SceneObject;
class SceneEntity;

class EntityRelation {
public:
    SceneEntity& entity1;
    SceneEntity& entity2;
};

class SceneUnderstanding {
public:
    const std::shared_ptr<PointCloud> cloud;
    std::vector<SceneObject> objects;
    std::vector<EntityRelation> relations;
    
    SceneUnderstanding(std::shared_ptr<PointCloud> cloud);
    auto object_clouds() -> std::vector<std::shared_ptr<PointCloud>>;
};
