#include "task_understanding.hpp"
#include "task_understanding.hpp"
#include "entity.hpp"
#include "object.hpp"
#include "properties.hpp"
#include "alignment.hpp"
#include "hole.hpp"
#include "peg.hpp"

using namespace std;
using boost::add_vertex;
using boost::vertices;
using boost::tie;
using boost::add_edge;
using boost::optional;


TaskUnderstanding::TaskUnderstanding(const shared_ptr<PointCloud> before_cloud,
                                     const shared_ptr<PointCloud> after_cloud):
before_scene(before_cloud),
after_scene(after_cloud)
{
    
}


auto TaskUnderstanding::detect_objects() -> void
{
    const auto clouds = ::detect_objects(before_scene.cloud);
    
    /* Segment out objects on before_scene */
    for (int i = 0; i < clouds.size(); ++i) {
        auto& c = clouds[i];
        auto props = detect_properties(c);
        auto oid = "object_" + to_string(i);
        Entity entity{Entity::Type::object, oid, props.position, props.rotation,
            c, props.min_corner, props.max_corner};
        add_vertex(entity, before_scene.graph);
    }
    
    /* Find the same objects in after_scene */
    VertexIter i, end;
    auto& graph = before_scene.graph;
    auto& after_cloud = after_scene.cloud;
    
    for (tie(i, end) = vertices(graph); i != end; ++i) {
        auto e = graph[*i];
        
        if (e.type != Entity::Type::object) {
            continue;
        }
        
        shared_ptr<PointCloud> aligned_cloud;
        Transform transform;
        tie(transform, aligned_cloud) = align(e.cloud, after_cloud);
        auto props = detect_properties(aligned_cloud);
        Entity entity{Entity::Type::object, e.id, props.position, props.rotation,
            aligned_cloud, props.min_corner, props.max_corner};
        add_vertex(entity, after_scene.graph);
        object_transforms.push_back(transform);
    }
}

auto TaskUnderstanding::detect_change() -> void
{
    focus_ids.push_back("object_0");
    focus_ids.push_back("object_1");
}

auto TaskUnderstanding::detect_features() -> void
{
    auto& graph = before_scene.graph;
    auto& after_graph = after_scene.graph;
    
    for (int i = 0; i < focus_ids.size(); ++i) {
        auto &fid = focus_ids[i];
        auto object_desc = find_vertex(fid, graph);
        auto object_cpy = find_vertex(fid, after_graph);
        auto& cpy_transform = object_transforms[i];
        
        if (!object_desc) {
            continue;
        }
        
        // TODO: Why this cannot be a reference?
        // Could be because vertex iterators invalidate when we add
        // elements while iterating...
        // Perhaps this will be fixed when we add features outside this loop
        // https://www.boost.org/doc/libs/1_77_0/libs/graph/doc/adjacency_list.html
        // Alternatively we could use find_vertex after each graph modification
        // Or just let it be a copy, why not?
        // Or use listS instead of vecS
        // No, actually I think the reference may be poiniting to a memory that was
        // changed when vector resized itself...
        // https://stackoverflow.com/questions/50867703/keeping-a-reference-to-a-vector-element-valid-after-resizing
        auto object = graph[*object_desc];
        
        /* Mass center */
        const auto props = detect_properties(object.cloud);
        Entity mass_center_entity {Entity::Type::mass_center, "mass_center", props.mass_center};
        add_feature(mass_center_entity, *object_desc, object_cpy, cpy_transform);
        
        
        /* Holes */
        auto holes = find_holes(object.cloud);
        add_features(holes, "hole_", Entity::Type::hole,
                     *object_desc,
                     object_cpy, cpy_transform);
        
        /* Pegs */
        const auto pegs = detect_pegs(object.cloud);
        add_features(pegs, "peg_", Entity::Type::peg,
                     *object_desc,
                     object_cpy, cpy_transform);
    }
}


auto TaskUnderstanding::add_features(const vector<shared_ptr<PointCloud>>& clouds,
                                     const string& prefix, const Entity::Type& type,
                                     const VertexDesc& vertex,
                                     const optional<VertexDesc>& vertex_cpy,
                                     const Transform& cpy_transform) -> void
{
    for (int i = 0; i < clouds.size(); ++i) {
        auto& c = clouds[i];
        auto props = detect_properties(c);
        
        
        // TODO: Somehow move this somewhere else
        if (type == Entity::Type::peg) {
            props.position.z() += 0.02;
            props.max_corner.z += 0.04;
        }
        
        auto pid = prefix + to_string(i);
        Entity entity{type, pid, props.position, props.rotation, c, props.min_corner, props.max_corner};
        add_feature(entity, vertex, vertex_cpy, cpy_transform);
    }
}

auto TaskUnderstanding::add_feature(const Entity& entity,
                                    const VertexDesc& vertex,
                                    const optional<VertexDesc>& vertex_cpy,
                                    const Transform& cpy_transform) -> void
{
    auto& graph = before_scene.graph;
    auto& after_graph = after_scene.graph;
    
    auto desc = add_vertex(entity, graph);
    add_edge(vertex, desc, graph);
    
    /* Copy featur to after_scene */
    if (vertex_cpy) {
        auto cpy_entity = entity.transformed(cpy_transform);
        auto cpy_desc = add_vertex(cpy_entity, after_graph);
        add_edge(*vertex_cpy, cpy_desc, after_graph);
    }
}

auto TaskUnderstanding::describe_relations() -> void
{
    before_scene.describe_relations(focus_ids);
    after_scene.describe_relations(focus_ids);
}
