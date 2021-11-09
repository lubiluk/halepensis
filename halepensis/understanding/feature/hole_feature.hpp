#pragma once

#include "geometry.hpp"
#include "scene_entity.hpp"
#include <memory>
#include <string>

class HoleFeature: public CloudEntity {
public:
    HoleFeature(std::string id,
                std::shared_ptr<PointCloud> cloud,
                Vector position,
                Quaternion orientation);
    HoleFeature(const HoleFeature& original);
    virtual auto clone() -> std::shared_ptr<SceneEntity> const override;
};
