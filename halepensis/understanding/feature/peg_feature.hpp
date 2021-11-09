#pragma once

#include "geometry.hpp"
#include "scene_entity.hpp"
#include <memory>
#include <string>


class PegFeature: public CloudEntity {
public:
    PegFeature(std::string id,
               std::shared_ptr<PointCloud> cloud,
               Vector position,
               Quaternion orientation);
    PegFeature(const PegFeature& original);
    virtual auto clone() -> std::shared_ptr<SceneEntity> const override;
};
