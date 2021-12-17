#include "scene_graph.hpp"

using namespace std;
using boost::optional;
using boost::tie;
using boost::vertices;
using boost::none;
using boost::adjacent_vertices;

auto find_object(const entity_id& id, const scene_graph& g)
-> optional<scene_graph::vertex_descriptor>
{
    scene_graph::vertex_iterator start, end;
    tie(start, end) = vertices(g);
    
    auto vertex = find_if(start, end, [&g, &id](auto vd) {
        return g[vd].id == id;
    });
    
    if (vertex == end) {
        return none;
    }
    
    return *vertex;
}

auto find_feature(const entity_id& obj_id, const entity_id& feature_id, const scene_graph& g)
-> optional<scene_graph::vertex_descriptor>
{
    auto obj_v = find_object(obj_id, g);
    
    if (!obj_v) {
        return none;
    }
    
    return find_feature(*obj_v, feature_id, g);
}

auto find_feature(scene_graph::vertex_descriptor obj_v, const entity_id& feature_id, const scene_graph& g)
-> optional<scene_graph::vertex_descriptor>
{
    scene_graph::adjacency_iterator start, end;
    tie(start, end) = adjacent_vertices(obj_v, g);
    
    auto vertex = find_if(start, end, [&g, &feature_id](auto vd) {
        return g[vd].id == feature_id;
    });
    
    if (vertex == end) {
        return none;
    }
    
    return *vertex;
}

auto object(scene_graph::vertex_descriptor feature_vertex, const scene_graph& g) -> scene_graph::vertex_descriptor
{
    scene_graph::inv_adjacency_iterator i, end;
    for (tie(i, end) = inv_adjacent_vertices(feature_vertex, g); i != end; ++i) {
        auto& obj = g[*i];
        if (obj.type == entity_type::object) {
            return *i;
        }
    }
    
    // hmm... theoretically impossible to end up here, but what if?
    return *end - 1;
}

auto objects(const scene_graph& g) -> vector<scene_entity>
{
    vector<scene_entity> objects;
    scene_graph::vertex_iterator i, end;
    
    for (tie(i, end) = vertices(g); i != end; ++i) {
        auto& obj = g[*i];
        if (obj.type == entity_type::object) {
            objects.push_back(obj);
        }
    }
    
    return objects;
}

auto features(const string& object_id, const scene_graph& g) -> vector<scene_graph::vertex_descriptor>
{
    auto v = find_object(object_id, g);
    
    if (!v) {
        return {};
    }
    
    vector<scene_graph::vertex_descriptor> features;
    scene_graph::adjacency_iterator i, end;
    for (tie(i, end) = adjacent_vertices(*v, g); i != end; ++i) {
        auto& obj = g[*i];
        if (obj.type != entity_type::object) {
            features.push_back(*i);
        }
    }
    
    return features;
}

auto features(const vector<string>& object_ids, const scene_graph& g) -> vector<scene_graph::vertex_descriptor>
{
    vector<scene_graph::vertex_descriptor> all_feats;
    
    for (auto& id : object_ids) {
        auto obj_feats = features(id, g);
        if (!obj_feats.empty()) {
            all_feats.insert(all_feats.end(), obj_feats.begin(), obj_feats.end());
        }
    }
    
    return all_feats;
}
