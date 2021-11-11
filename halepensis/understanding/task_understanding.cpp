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
        Entity entity{Entity::Type::object, oid, props.position, props.rotation, c};
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
        Entity entity{Entity::Type::object, e.id, props.position, props.rotation, aligned_cloud};
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
        
        auto& object = graph[*object_desc];
        
        /* Mass center */
        const auto props = detect_properties(object.cloud);
        Entity mass_center_entity {Entity::Type::mass_center, "mass_center", props.mass_center};
        auto mass_center_desc = add_vertex(mass_center_entity, graph);
        add_edge(*object_desc, mass_center_desc, graph);
        
        /* Copy featur to after_scene */
        if (object_cpy) {
            auto cpy_entity = mass_center_entity.transformed(cpy_transform);
            auto cpy_desc = add_vertex(cpy_entity, after_graph);
            add_edge(*object_cpy, cpy_desc, after_graph);
        }
        
        
        /* Holes */
        auto holes = find_holes(object.cloud);
        for (int i = 0; i < holes.size(); ++i) {
            auto& h = holes[i];
            auto props = detect_properties(h);
            auto hid = "hole_" + to_string(i);
            Entity hole_entity{Entity::Type::hole, hid, props.position, props.rotation, h};
            auto hole_desc = add_vertex(hole_entity, graph);
            add_edge(*object_desc, hole_desc, graph);
            
            /* Copy featur to after_scene */
            if (object_cpy) {
                auto cpy_entity = hole_entity.transformed(cpy_transform);
                auto cpy_desc = add_vertex(cpy_entity, after_graph);
                add_edge(*object_cpy, cpy_desc, after_graph);
            }
        }
        
        /* Pegs */
        const auto pegs = detect_pegs(object.cloud);
        for (int i = 0; i < pegs.size(); ++i) {
            auto& p = pegs[i];
            auto props = detect_properties(p);
            auto pid = "peg_" + to_string(i);
            Entity peg_entity{Entity::Type::peg, pid, props.position, props.rotation, p};
            auto peg_desc = add_vertex(peg_entity, graph);
            add_edge(*object_desc, peg_desc, graph);
            
            /* Copy featur to after_scene */
            if (object_cpy) {
                auto cpy_entity = peg_entity.transformed(cpy_transform);
                auto cpy_desc = add_vertex(cpy_entity, after_graph);
                add_edge(*object_cpy, cpy_desc, after_graph);
            }
        }
    }
}
