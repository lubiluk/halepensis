#include "task_rule.hpp"
#include <map>

using std::map;
using std::string;
using std::set;
using std::vector;
using boost::add_vertex;
using boost::add_edge;

bool operator==(const task_rule& lhs, const task_rule& rhs)
{
    return std::tie(lhs.object1_id, lhs.feature1_id, lhs.relation_type, lhs.object2_id, lhs.feature2_id)
    == std::tie(rhs.object1_id, rhs.feature1_id, rhs.relation_type, rhs.object2_id, rhs.feature2_id);
}

bool operator<(const task_rule& lhs, const task_rule& rhs)
{
    return std::tie(lhs.object1_id, lhs.feature1_id, lhs.relation_type, lhs.object2_id, lhs.feature2_id)
    < std::tie(rhs.object1_id, rhs.feature1_id, rhs.relation_type, rhs.object2_id, rhs.feature2_id);
}

auto find_or_copy_object(const entity_id& obj_id, scene_graph& graph, const scene_graph& scene)
-> scene_graph::vertex_descriptor
{
    auto obj_v = find_object(obj_id, graph);
    if (!obj_v) {
        auto obj_src_v = find_object(obj_id, scene);
        obj_v = add_vertex(scene[*obj_src_v], graph);
    }
    
    return *obj_v;
}

auto find_or_create_object(const entity_id& obj_id, scene_graph& graph)
-> scene_graph::vertex_descriptor
{
    auto obj_v = find_object(obj_id, graph);
    if (!obj_v) {
        obj_v = add_vertex({entity_type::object, obj_id, {0.0f, 0.0f, 0.0f}}, graph);
    }
    
    return *obj_v;
}

auto find_or_copy_feature(scene_graph::vertex_descriptor obj_v, const entity_id& feat_id,
                          scene_graph& graph, const scene_graph& scene)
-> scene_graph::vertex_descriptor
{
    auto feat_v = find_feature(obj_v, feat_id, graph);
    if (!feat_v) {
        auto feat_src_v = find_feature(graph[obj_v].id, feat_id, scene);
        feat_v = add_vertex(scene[*feat_src_v], graph);
        add_edge(obj_v, *feat_v, relation_type::has, graph);
    }
    
    return *feat_v;
}

auto find_or_create_feature(scene_graph::vertex_descriptor obj_v, const entity_id& feat_id,
                          scene_graph& graph)
-> scene_graph::vertex_descriptor
{
    auto feat_v = find_feature(obj_v, feat_id, graph);
    if (!feat_v) {
        feat_v = add_vertex({*entity_type_from_string(feat_id), feat_id, {0.0f, 0.0f, 0.0f}}, graph);
        add_edge(obj_v, *feat_v, relation_type::has, graph);
    }
    
    return *feat_v;
}

auto graph_from_rules(set<task_rule> rules, scene_graph scene) -> scene_graph
{
    map<string, scene_graph::vertex_descriptor> nodes;
    scene_graph graph;
    
    for (auto& r : rules) {
        auto obj1_v = find_or_copy_object(r.object1_id, graph, scene);
        auto feat1_v = find_or_copy_feature(obj1_v, r.feature1_id, graph, scene);
        auto obj2_v = find_or_copy_object(r.object2_id, graph, scene);
        auto feat2_v = find_or_copy_feature(obj2_v, r.feature2_id, graph, scene);
        add_edge(feat1_v, feat2_v, r.relation_type, graph);
    }
    
    return graph;
}

auto graph_from_rules(set<task_rule> rules) -> scene_graph
{
    map<string, scene_graph::vertex_descriptor> nodes;
    scene_graph graph;
    
    for (auto& r : rules) {
        auto obj1_v = find_or_create_object(r.object1_id, graph);
        auto feat1_v = find_or_create_feature(obj1_v, r.feature1_id, graph);
        auto obj2_v = find_or_create_object(r.object2_id, graph);
        auto feat2_v = find_or_create_feature(obj2_v, r.feature2_id, graph);
        add_edge(feat1_v, feat2_v, r.relation_type, graph);
    }
    
    return graph;
}

auto rules_from_features(const vector<scene_graph::vertex_descriptor>& feats,
                         const scene_graph& g,
                         bool use_feature_ids) -> set<task_rule>
{
    using boost::tie;
    set<task_rule> rules;
    
    for (auto v : feats) {
        scene_graph::out_edge_iterator it, end;
        for (tie(it, end) = out_edges(v, g); it != end; ++it) {
            auto feat1_v = source(*it, g);
            auto feat2_v = target(*it, g);
            auto obj1_v = object(feat1_v, g);
            auto obj2_v = object(feat2_v, g);
            rules.emplace(g[obj1_v].id,
                          use_feature_ids ? g[feat1_v].id : g[feat1_v].type_description(),
                          g[*it].description(),
                          g[obj2_v].id,
                          use_feature_ids ? g[feat2_v].id : g[feat2_v].type_description());
        }
    }
    
    return rules;
}

auto rules_from_graph(const scene_graph& g, bool use_feature_ids) -> set<task_rule>
{
    using boost::tie;
    
    vector<scene_graph::vertex_descriptor> feats;
    scene_graph::vertex_iterator i, end;
    
    for (tie(i, end) = vertices(g); i != end; ++i) {
        auto& ent = g[*i];
        if (ent.type != entity_type::object) {
            feats.push_back(*i);
        }
    }
    
    return rules_from_features(feats, g, use_feature_ids);
}
