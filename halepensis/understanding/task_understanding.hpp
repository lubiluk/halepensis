#pragma once

#include "geometry.hpp"
#include "scene_understanding.hpp"
#include <memory>
#include <vector>
#include <boost/optional.hpp>

class TaskUnderstanding {
public:
    SceneUnderstanding before_scene;
    SceneUnderstanding after_scene;
    std::vector<std::string> focus_ids;
    std::vector<Transform> object_transforms;
    
    TaskUnderstanding(std::shared_ptr<PointCloud> before_cloud, const std::shared_ptr<PointCloud> after_cloud);
    
    
    auto detect_objects() -> void;
    /// Detects which objects are involved in the task and fills in focus_descriptors in the task.
    auto detect_change() -> void;
    /// Detects features in focused entities
    auto detect_features() -> void;
    auto describe_relations() -> void;
//    auto form_hypotheses() -> void;
private:
    auto add_features(const std::vector<std::shared_ptr<PointCloud>>& clouds,
                      const std::string& prefix, const Entity::Type& type,
                      const VertexDesc& vertex,
                      const boost::optional<VertexDesc>& vertex_cpy,
                      const Transform& cpy_transform) -> void;
    auto add_feature(const Entity& entity,
                     const VertexDesc& vertex,
                     const boost::optional<VertexDesc>& vertex_cpy,
                     const Transform& cpy_transform) -> void;
};
