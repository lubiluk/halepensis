#include "task_understanding.hpp"
#include "task_understanding.hpp"
#include "scene_entity.hpp"
#include "object.hpp"
#include "properties.hpp"
#include "alignment.hpp"
#include "hole.hpp"
#include "peg.hpp"
#include "task_rule.hpp"
#include <tuple>
#include <set>
#include <utility>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/common/transforms.h>
#pragma clang diagnostic pop

using namespace std;
using boost::add_vertex;
using boost::vertices;
using boost::tie;
using boost::add_edge;
using boost::optional;
using pcl::transformPointCloud;


task_understanding::task_understanding(const shared_ptr<point_cloud> before_cloud,
                                     const shared_ptr<point_cloud> after_cloud):
before_scene(before_cloud),
after_scene(after_cloud)
{
    
}


auto task_understanding::detect_objects(map<string, mat44> given_transforms) -> void
{
    const auto clouds = ::detect_objects(before_scene.cloud);
    
    /* Segment out objects on before_scene */
    for (int i = 0; i < clouds.size(); ++i) {
        auto& c = clouds[i];
        auto props = detect_properties(c);
        auto oid = "object_" + to_string(i);
        scene_entity entity{entity_type::object, oid, props.position, props.rotation,
            c, props.min_corner, props.max_corner};
        add_vertex(entity, before_scene.graph);
    }
    
    /* Find the same objects in after_scene */
    scene_graph::vertex_iterator i, end;
    auto& graph = before_scene.graph;
    auto& after_cloud = after_scene.cloud;
    
    for (tie(i, end) = vertices(graph); i != end; ++i) {
        auto entity = graph[*i];
        
        if (entity.type != entity_type::object) {
            continue;
        }
        
        shared_ptr<point_cloud> aligned_cloud;
        mat44 transform;
        
        if (given_transforms.find(entity.id) != given_transforms.end()) {
            aligned_cloud = make_shared<point_cloud>();
            transform = given_transforms[entity.id];
            transformPointCloud(*entity.cloud, *aligned_cloud, transform);
        } else {
            tie(transform, aligned_cloud) = align(entity.cloud, after_cloud);
        }
        
        scene_entity after_entity = entity.transformed(transform);
        add_vertex(after_entity, after_scene.graph);
        
        object_transforms[entity.id] = transform;
    }
}

auto task_understanding::detect_change() -> void
{
    // Displaced objects
    for (auto& be : objects(before_scene.graph)) {
        if (auto aov = find_object(be.id, after_scene.graph)) {
            auto &ae = after_scene.graph[*aov];
            
            if ((ae.position - be.position).norm() > 0.02 ) {
                focus_ids.push_back(be.id);
            }
        }
    }
    
    // Objects that are close to displaced objects
    for (auto& fid : focus_ids) {
        auto fov = find_object(fid, after_scene.graph);
        auto &fe = after_scene.graph[*fov];
        
        for (auto& ae : objects(after_scene.graph)) {
            if (fe.id == ae.id) {
                continue;
            }
            
            auto fe_rad = (point_to_vec3(fe.max_corner) - point_to_vec3(fe.min_corner)).norm() / 2.0;
            auto ae_rad = (point_to_vec3(ae.max_corner) - point_to_vec3(ae.min_corner)).norm() / 2.0;
            
            if ((fe.position - ae.position).norm() <= (fe_rad + ae_rad)) {
                focus_ids.push_back(ae.id);
            }
        }
        
    }
}

auto task_understanding::detect_features() -> void
{
    auto& graph = before_scene.graph;
    auto& after_graph = after_scene.graph;
    
    for (int i = 0; i < focus_ids.size(); ++i) {
        auto &fid = focus_ids[i];
        auto object_desc = find_object(fid, graph);
        auto object_cpy = find_object(fid, after_graph);
        auto& cpy_transform = object_transforms[fid];
        
        if (!object_desc) {
            continue;
        }
        
        // Why this cannot be a reference:
        // https://stackoverflow.com/questions/50867703/keeping-a-reference-to-a-vector-element-valid-after-resizing
        auto object = graph[*object_desc];
        
        /* Mass center */
        const auto props = detect_properties(object.cloud);
        scene_entity mass_center_entity {entity_type::mass_center, "mass_center", props.mass_center};
        add_feature(mass_center_entity, *object_desc, object_cpy, cpy_transform);
        
        
        /* Holes */
        auto holes = detect_holes(object.cloud);
        add_features(holes, "hole_", entity_type::hole,
                     *object_desc,
                     object_cpy, cpy_transform);
        
        /* Pegs */
        const auto pegs = detect_pegs(object.cloud);
        add_features(pegs, "peg_", entity_type::peg,
                     *object_desc,
                     object_cpy, cpy_transform);
    }
}


auto task_understanding::add_features(const vector<shared_ptr<point_cloud>>& clouds,
                                     const string& prefix, const entity_type& type,
                                     const scene_graph::vertex_descriptor& vertex,
                                     const optional<scene_graph::vertex_descriptor>& vertex_cpy,
                                     const mat44& cpy_transform) -> void
{
    for (int i = 0; i < clouds.size(); ++i) {
        auto& c = clouds[i];
        auto props = detect_properties(c);
        
        
        // TODO: Somehow move this somewhere else
        if (type == entity_type::peg) {
            props.position.z() += 0.02;
            props.max_corner.z += 0.04;
        }
        
        auto pid = prefix + to_string(i);
        scene_entity entity{type, pid, props.position, props.rotation, c, props.min_corner, props.max_corner};
        add_feature(entity, vertex, vertex_cpy, cpy_transform);
    }
}

auto task_understanding::add_feature(const scene_entity& entity,
                                    const scene_graph::vertex_descriptor& vertex,
                                    const optional<scene_graph::vertex_descriptor>& vertex_cpy,
                                    const mat44& cpy_transform) -> void
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

auto task_understanding::describe_relations() -> void
{
    before_scene.describe_relations(focus_ids);
    after_scene.describe_relations(focus_ids);
}

auto task_understanding::describe_task() -> void
{
    auto before_feats = features(focus_ids, before_scene.graph);
    auto after_feats = features(focus_ids, after_scene.graph);
    
    set<task_rule> before_rules = rules_from_features(before_feats, before_scene.graph);
    set<task_rule> after_rules = rules_from_features(after_feats, after_scene.graph);
    
    set<task_rule> created_rules;
    set_difference(after_rules.begin(), after_rules.end(),
                   before_rules.begin(), before_rules.end(),
                   inserter(created_rules, created_rules.begin()));
    
//    cout << "Created rules:" << endl;
//    for (auto &r : created_rules)  {
//        cout << r.object1_id << " - " << r.feature1_id
//        << " --" << r.relation_type << "--> "
//        << r.object2_id << " - " << r.feature2_id << endl;
//    }
//    
    task_description = graph_from_rules(created_rules, after_scene.graph);
}
