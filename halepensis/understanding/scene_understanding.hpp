#pragma once

#include "geometry.hpp"
#include "scene_entity.hpp"
#include "entity_relation.hpp"
#include "scene_graph.hpp"
#include <string>
#include <vector>
#include <memory>
#include <boost/optional.hpp>

class scene_understanding {
public:
    std::shared_ptr<point_cloud> cloud;
    scene_graph graph;
    
    scene_understanding(std::shared_ptr<point_cloud> cloud);
    auto object_clouds() const -> std::vector<std::shared_ptr<point_cloud>>;
    auto describe_relations(const std::vector<std::string>& object_ids, bool use_hack = false) -> void;
};
