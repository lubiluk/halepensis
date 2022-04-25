#pragma once

#include "geometry.hpp"
#include "scene_understanding.hpp"
#include "scene_graph.hpp"
#include <memory>
#include <vector>
#include <boost/optional.hpp>

class task_understanding {
public:
    scene_understanding before_scene;
    scene_understanding after_scene;
    std::vector<std::string> focus_ids;
    std::vector<mat44> object_transforms;
    scene_graph task_description;
    
    task_understanding(std::shared_ptr<point_cloud> before_cloud, const std::shared_ptr<point_cloud> after_cloud);
    
    
    auto detect_objects(bool use_hack = false) -> void;
    /// Detects which objects are involved in the task and fills in focus_descriptors in the task.
    auto detect_change() -> void;
    /// Detects features in focused entities
    auto detect_features() -> void;
    /// Describes relations between features of focused objects
    auto describe_relations(bool use_hack = false) -> void;
    auto describe_task() -> void;
private:
    auto add_features(const std::vector<std::shared_ptr<point_cloud>>& clouds,
                      const std::string& prefix, const entity_type& type,
                      const scene_graph::vertex_descriptor& vertex,
                      const boost::optional<scene_graph::vertex_descriptor>& vertex_cpy,
                      const mat44& cpy_transform) -> void;
    auto add_feature(const scene_entity& entity,
                     const scene_graph::vertex_descriptor& vertex,
                     const boost::optional<scene_graph::vertex_descriptor>& vertex_cpy,
                     const mat44& cpy_transform) -> void;
};
