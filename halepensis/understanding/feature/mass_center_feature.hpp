#pragma once

#include "geometry.hpp"
#include "scene_entity.hpp"
#include <memory>
#include <string>


class MassCenterFeature: public SceneEntity {
public:
    MassCenterFeature(std::string id, Vector position);
    virtual auto clone() -> std::shared_ptr<SceneEntity> const override;
};
