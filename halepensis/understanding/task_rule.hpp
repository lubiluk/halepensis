#pragma once

#include "scene_graph.hpp"
#include <string>
#include <set>

struct task_rule {
    std::string object1_id;
    std::string feature1_id;
    std::string relation_type;
    std::string object2_id;
    std::string feature2_id;
    
    task_rule(std::string object1_id,
              std::string feature1_id,
              std::string relation_type,
              std::string object2_id,
              std::string feature2_id):
    object1_id(object1_id),
    feature1_id(feature1_id),
    relation_type(relation_type),
    object2_id(object2_id),
    feature2_id(feature2_id)
    {
        
    }
};

bool operator==(const task_rule& lhs, const task_rule& rhs);

bool operator<(const task_rule& lhs, const task_rule& rhs);

auto graph_from_rules(std::set<task_rule> rules, scene_graph scene) -> scene_graph;

auto graph_from_rules(std::set<task_rule> rules) -> scene_graph;

auto rules_from_features(const std::vector<scene_graph::vertex_descriptor>& feats,
                         const scene_graph& g,
                         bool use_feature_ids = true)
-> std::set<task_rule>;

auto rules_from_graph(const scene_graph& g, bool use_feature_ids = true) -> std::set<task_rule>;
