#include "scene_understanding.hpp"

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

auto SceneUnderstanding::features(const Entity& object) const -> vector<Entity>
{
    auto v = find_vertex(object.id, graph);
    
    if (!v) {
        return {};
    }
    
    vector<Entity> features;
    AdjacencyIter i, end;
    for (tie(i, end) = adjacent_vertices(*v, graph); i != end; ++i) {
        auto& obj = graph[*i];
        if (obj.type != Entity::Type::object) {
            features.push_back(obj);
        }
    }
    
    return features;
}
