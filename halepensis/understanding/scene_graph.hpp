#pragma once

#include "geometry.hpp"
#include <memory>
#include <vector>
#include <string>
#include <map>

// Entity exists, hence must have a position. Orientation might be meaningless though.
class SceneEntity {
public:
    const std::string id;
    const Vector position;
    const Quaternion orientation;
    virtual ~SceneEntity() = 0;
    
protected:
    SceneEntity(const std::string id,
                const Vector position,
                const Quaternion orientation = Quaternion{});
};

class CloudEntity: public SceneEntity {
public:
    const std::shared_ptr<PointCloud> cloud;
    virtual ~CloudEntity() = 0;
protected:
    CloudEntity(const std::string id,
                const std::shared_ptr<PointCloud> cloud,
                const Vector position,
                const Quaternion orientation);
};

class HoleFeature: public CloudEntity {
public:
    HoleFeature(const std::shared_ptr<PointCloud> cloud,
                const Vector position,
                const Quaternion orientation);
};

class MassCenter: public SceneEntity {
public:
    MassCenter(const Vector position);
};

class SceneObject: public CloudEntity {
public:
    std::vector<std::shared_ptr<SceneEntity>> features;
    
    SceneObject(const std::shared_ptr<PointCloud> cloud,
           const Vector position,
           const Quaternion orientation);
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
    
    SceneUnderstanding(const std::shared_ptr<PointCloud> cloud);
    auto object_clouds() -> std::vector<std::shared_ptr<PointCloud>>;
};

class TaskUnderstanding {
public:
    SceneUnderstanding before_scene;
    SceneUnderstanding after_scene;
    std::vector<std::vector<SceneObject>::size_type> focus_indices;
    
    TaskUnderstanding(const std::shared_ptr<PointCloud> before_cloud, const std::shared_ptr<PointCloud> after_cloud);
};
