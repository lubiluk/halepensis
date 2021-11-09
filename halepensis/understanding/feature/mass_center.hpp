#pragma once

#include "geometry.hpp"
#include "scene_entity.hpp"
#include <memory>
#include <string>

class MassCenter: public SceneEntity {
public:
    MassCenter(std::string id, Vector position);
    virtual auto clone() -> std::shared_ptr<SceneEntity> const override;
};
