#include "scene_understanding.hpp"
#include "relation_detection.hpp"

using std::string;
using std::shared_ptr;
using std::vector;
using std::back_inserter;
using std::tuple;
using std::get;
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
//            if (is_inside(e1, e2)) {
//                add_edge(v1, v2, relation_type::inside, graph);
//            }
//            if (is_inside(e2, e1)) {
//                add_edge(v2, v1, relation_type::inside, graph);
//            }
            
            /* Touching */
            if (is_touching(e1, e2)) {
                add_edge(v1, v2, relation_type::touching, graph);
            }
        }
    }
}

auto scene_understanding::add_relation(relation_rule given_relation) -> void
{
    auto& obj1_id = get<0>(given_relation);
    auto& feat1_id = get<1>(given_relation);
    auto& relation = get<2>(given_relation);
    auto& obj2_id = get<3>(given_relation);
    auto& feat2_id = get<4>(given_relation);
    
    auto v1 = find_feature(obj1_id, feat1_id, graph);
    auto v2 = find_feature(obj2_id, feat2_id, graph);
    
    add_edge(*v1, *v2, relation, graph);
}

