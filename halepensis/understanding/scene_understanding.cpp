#include "scene_understanding.hpp"
#include "below.hpp"

using std::string;
using std::shared_ptr;
using std::vector;
using std::back_inserter;
using boost::tie;
using boost::adjacent_vertices;

SceneUnderstanding::SceneUnderstanding(const shared_ptr<PointCloud> cloud):
cloud(cloud)
{
    
}

auto SceneUnderstanding::object_clouds() const -> vector<shared_ptr<PointCloud>>
{
    SceneGraph::vertex_iterator i, end;
    vector<shared_ptr<PointCloud>> clouds;
    
    for (tie(i, end) = vertices(graph); i != end; ++i) {
        clouds.push_back(graph[*i].cloud);
    }
    
    return clouds;
}

auto SceneUnderstanding::objects() const -> vector<Entity>
{
    vector<Entity> objects;
    VertexIter i, end;
    
    for (tie(i, end) = vertices(graph); i != end; ++i) {
        auto& obj = graph[*i];
        if (obj.type == Entity::Type::object) {
            objects.push_back(obj);
        }
    }
    
    return objects;
}

auto SceneUnderstanding::features(const string& object_id) const -> vector<VertexDesc>
{
    auto v = find_vertex(object_id, graph);
    
    if (!v) {
        return {};
    }
    
    vector<VertexDesc> features;
    AdjacencyIter i, end;
    for (tie(i, end) = adjacent_vertices(*v, graph); i != end; ++i) {
        auto& obj = graph[*i];
        if (obj.type != Entity::Type::object) {
            features.push_back(*i);
        }
    }
    
    return features;
}

auto SceneUnderstanding::features(const vector<string>& object_ids) const -> vector<VertexDesc>
{
    vector<VertexDesc> all_feats;
    
    for (auto& id : object_ids) {
        auto obj_feats = features(id);
        if (!obj_feats.empty()) {
            all_feats.insert(all_feats.end(), obj_feats.begin(), obj_feats.end());
        }
    }
    
    return all_feats;
}

auto SceneUnderstanding::describe_relations(const vector<string> &object_ids) -> void
{
    auto fts = features(object_ids);
    
    // Go through features pairwise
    for (auto it = fts.begin(); it != fts.end() - 1; ++it) {
        auto v1 = *it;
        for (auto jt = it + 1; jt != fts.end(); ++ jt) {
            auto v2 = *jt;
            
            auto& e1 = graph[v1];
            auto& e2 = graph[v2];
            
            /* Below */
            if (is_below(e1, e2)) {
                add_edge(v1, v2, Relation::Type::below, graph);
            }
            if (is_below(e2, e1)) {
                add_edge(v2, v1, Relation::Type::below, graph);
            }
        }
    }
}
