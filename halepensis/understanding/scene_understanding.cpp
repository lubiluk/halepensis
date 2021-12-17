#include "scene_understanding.hpp"
#include "relation_detection.hpp"

using std::string;
using std::shared_ptr;
using std::vector;
using std::back_inserter;
using boost::tie;
using boost::adjacent_vertices;
using boost::optional;
using boost::none;

scene_understanding::scene_understanding(shared_ptr<point_cloud> cloud):
cloud(cloud)
{
    
}

auto scene_understanding::object_clouds() const -> vector<shared_ptr<point_cloud>>
{
    scene_graph::vertex_iterator i, end;
    vector<shared_ptr<point_cloud>> clouds;
    
    for (tie(i, end) = vertices(graph); i != end; ++i) {
        clouds.push_back(graph[*i].cloud);
    }
    
    return clouds;
}

auto scene_understanding::describe_relations(const vector<string> &object_ids) -> void
{
    auto fts = features(object_ids, graph);
    
    // Go through features pairwise
    for (auto it = fts.begin(); it != fts.end() - 1; ++it) {
        auto v1 = *it;
        for (auto jt = it + 1; jt != fts.end(); ++ jt) {
            auto v2 = *jt;
            
            auto& e1 = graph[v1];
            auto& e2 = graph[v2];
            
            /* Below */
            if (is_below(e1, e2)) {
                add_edge(v1, v2, relation_type::below, graph);
            }
            if (is_below(e2, e1)) {
                add_edge(v2, v1, relation_type::below, graph);
            }
            
            /* Inside */
            if (is_inside(e1, e2)) {
                add_edge(v1, v2, relation_type::inside, graph);
            }
            if (is_inside(e2, e1)) {
                add_edge(v2, v1, relation_type::inside, graph);
            }
        }
    }
}

