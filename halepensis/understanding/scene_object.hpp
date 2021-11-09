#pragma once

#include "geometry.hpp"
#include "scene_entity.hpp"
#include <memory>
#include <string>
#include <vector>

class SceneObject: public CloudEntity {
public:
    std::vector<std::shared_ptr<SceneEntity>> features;
    
    SceneObject(std::string id,
                std::shared_ptr<PointCloud> cloud,
                Vector position,
                Quaternion orientation);
    SceneObject(const SceneObject& original);
    virtual auto clone() -> std::shared_ptr<SceneEntity> const override;
};
