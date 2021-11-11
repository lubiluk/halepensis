#pragma once

#include "geometry.hpp"
#include "scene_understanding.hpp"
#include <memory>
#include <vector>

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
//    auto describe_relations() -> void;
//    auto form_hypotheses() -> void;
//private:
//    auto copy_features() -> void;
};
