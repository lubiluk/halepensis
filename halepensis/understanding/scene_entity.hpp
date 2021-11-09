#pragma once

#include "geometry.hpp"
#include <memory>
#include <vector>
#include <string>
#include <map>


/**
 * Entity exists, hence must have a position. Orientation might be meaningless though.
 */
class SceneEntity {
public:
    std::string id;
    Vector position;
    Quaternion orientation;
    virtual ~SceneEntity() = 0;
    virtual auto transform(const Transform& transform) -> void;
    virtual auto clone() -> std::shared_ptr<SceneEntity> const = 0;
    
protected:
    SceneEntity(std::string id,
                Vector position,
                Quaternion orientation = Quaternion{});
};

/**
 * Entity that is physical, visible, has point cloud.
 */
class CloudEntity: public SceneEntity {
public:
    std::shared_ptr<PointCloud> cloud;
    virtual ~CloudEntity() = 0;
    virtual auto transform(const Transform& transform) -> void override;
protected:
    CloudEntity(std::string id,
                std::shared_ptr<PointCloud> cloud,
                Vector position,
                Quaternion orientation);
    CloudEntity(const CloudEntity& original);
};
