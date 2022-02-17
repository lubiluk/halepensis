#include "scene_graph.hpp"

#include <boost/graph/mcgregor_common_subgraphs.hpp>
#include <boost/graph/copy.hpp>

using namespace std;
using boost::optional;
using boost::tie;
using boost::vertices;
using boost::none;
using boost::adjacent_vertices;
using boost::property_map;
using boost::shared_array_property_map;
using boost::membership_filtered_graph_traits;
using boost::vertex_index_t;
using boost::vertex_index;
using boost::fill_membership_map;
using boost::mcgregor_common_subgraphs_maximum_unique;

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

template <typename Graph, typename Container>
struct collect_callback {
    collect_callback(const Graph& graph1,
                     const Graph& graph2,
                     Container& container) :
    graph1(graph1), graph2(graph2), container(container) { }
    
    template <typename CorrespondenceMapFirstToSecond, typename CorrespondenceMapSecondToFirst>
    bool operator()(CorrespondenceMapFirstToSecond correspondence_map_1_to_2,
                    CorrespondenceMapSecondToFirst correspondence_map_2_to_1,
                    typename boost::graph_traits<Graph>::vertices_size_type subgraph_size) {
        // Print out correspondences between vertices
        //        BGL_FORALL_VERTICES_T(vertex1, m_graph1, Graph) {
        //            // Skip unmapped vertices
        //            if (get(correspondence_map_1_to_2, vertex1) != boost::graph_traits<Graph>::null_vertex()) {
        //                std::cout << vertex1 << " <-> " << get(correspondence_map_1_to_2, vertex1) << std::endl;
        //            }
        //            auto v1 = m_graph1[vertex1];
        //            auto v2 = m_graph2[get(correspondence_map_1_to_2, vertex1)];
        //            std::cout << v1.id << " : " <<  v2.id << std::endl;
        //        }
        //
        //        std::cout << "---" << std::endl;
        
        typedef typename property_map<Graph, vertex_index_t>::type VertexIndexMap;
        typedef shared_array_property_map<bool, VertexIndexMap> MembershipMap;
        
        MembershipMap membership_map1(num_vertices(graph1),
                                      get(vertex_index, graph1));
        
        fill_membership_map<Graph>(graph1, correspondence_map_1_to_2, membership_map1);
        
        // Generate filtered graphs using membership map
        typedef typename membership_filtered_graph_traits<Graph, MembershipMap>::graph_type
        MembershipFilteredGraph;
        
        MembershipFilteredGraph subgraph1 =
        make_membership_filtered_graph(graph1, membership_map1);
        
        scene_graph sub_graph;
        copy_graph(subgraph1, sub_graph);
        container.push_back(sub_graph);
        
        return (true);
    }
    
    
private:
    const Graph& graph1;
    const Graph& graph2;
    Container& container;
};

auto intersection(const scene_graph& g1, const scene_graph& g2) -> scene_graph
{
    vector<scene_graph> sub_graphs;
    collect_callback<scene_graph, vector<scene_graph>> callback(g1, g2, sub_graphs);
    mcgregor_common_subgraphs_maximum_unique(g1, g2, true, callback);
    
    if (sub_graphs.empty()) {
        return {};
    }
    
    return sub_graphs.front();
}
