#pragma once

#include "geometry.hpp"
#include "scene_understanding.hpp"
#include <memory>
#include <vector>

class SceneObject;

class TaskUnderstanding {
public:
    SceneUnderstanding before_scene;
    SceneUnderstanding after_scene;
    std::vector<std::vector<SceneObject>::size_type> focus_indices;
    std::vector<Transform> object_transforms;
    
    TaskUnderstanding(std::shared_ptr<PointCloud> before_cloud, const std::shared_ptr<PointCloud> after_cloud);
};
