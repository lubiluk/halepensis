#pragma once

#include "geometry.hpp"
#include <memory>
#include <vector>
#include <string>
#include <map>

// Entity exists, hence must have a position. Orientation might be meaningless though.
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

class HoleFeature: public CloudEntity {
public:
    HoleFeature(std::string id,
                std::shared_ptr<PointCloud> cloud,
                Vector position,
                Quaternion orientation);
    HoleFeature(const HoleFeature& original);
    virtual auto clone() -> std::shared_ptr<SceneEntity> const override;
};

class MassCenter: public SceneEntity {
public:
    MassCenter(std::string id, Vector position);
    virtual auto clone() -> std::shared_ptr<SceneEntity> const override;
};

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

class TaskUnderstanding {
public:
    SceneUnderstanding before_scene;
    SceneUnderstanding after_scene;
    std::vector<std::vector<SceneObject>::size_type> focus_indices;
    std::vector<Transform> object_transforms;
    
    TaskUnderstanding(std::shared_ptr<PointCloud> before_cloud, const std::shared_ptr<PointCloud> after_cloud);
};
