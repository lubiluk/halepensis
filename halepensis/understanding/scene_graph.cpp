#include "scene_graph.hpp"

using namespace std;
using boost::optional;
using boost::tie;
using boost::vertices;
using boost::none;

auto find_vertex(const std::string& id, const SceneGraph& g) -> optional<VertexDesc>
{
    VertexIter start, end;
    tie(start, end) = vertices(g);
    
    auto vertex = find_if(start, end, [&g, &id](auto vd) {
        return g[vd].id == id;
    });
    
    if (vertex == end) {
        return none;
    }
    
    return *vertex;
}
