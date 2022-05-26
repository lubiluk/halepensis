#pragma once

#include "geometry.hpp"
#include "scene_entity.hpp"
#include "entity_relation.hpp"
#include "scene_graph.hpp"
#include <string>
#include <vector>
#include <memory>
#include <map>
#include <tuple>
#include <boost/optional.hpp>

using relation_rule = std::tuple<std::string, std::string, relation_type, std::string, std::string>;

class scene_understanding {
public:
    std::shared_ptr<point_cloud> cloud;
    scene_graph graph;
    
    scene_understanding(std::shared_ptr<point_cloud> cloud);
    auto object_clouds() const -> std::vector<std::shared_ptr<point_cloud>>;
    auto describe_relations(const std::vector<std::string>& object_ids) -> void;
    auto add_relation(relation_rule given_relations) -> void;
};
