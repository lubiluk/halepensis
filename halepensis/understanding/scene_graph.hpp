#pragma once

#include "scene_entity.hpp"
#include "entity_relation.hpp"
#include <string>


#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <boost/graph/adjacency_list.hpp>
#include <boost/optional.hpp>
#pragma clang diagnostic pop

using scene_graph = boost::adjacency_list
<boost::vecS, boost::vecS, boost::bidirectionalS, scene_entity, entity_relation>;

auto find_object(const entity_id& id, const scene_graph& g)
-> boost::optional<scene_graph::vertex_descriptor>;
auto find_feature(const entity_id& obj_id, const entity_id& feature_id, const scene_graph& g)
-> boost::optional<scene_graph::vertex_descriptor>;

auto find_feature(scene_graph::vertex_descriptor obj_v, const entity_id& feature_id, const scene_graph& g)
-> boost::optional<scene_graph::vertex_descriptor>;

auto object(scene_graph::vertex_descriptor feature_vertex, const scene_graph& g) -> scene_graph::vertex_descriptor;

auto objects(const scene_graph& g) -> std::vector<scene_entity>;

auto features(const std::string& object_id, const scene_graph& g) -> std::vector<scene_graph::vertex_descriptor>;

auto features(const std::vector<std::string>& object_ids, const scene_graph& g) -> std::vector<scene_graph::vertex_descriptor>;

auto intersection(const scene_graph& g1, const scene_graph& g2) -> std::vector<scene_graph>;

auto save_to_graphviz(const scene_graph& graph, const std::string& file_path) -> void;
