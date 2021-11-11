//#include "understanding.hpp"
//#include "task_understanding.hpp"
//#include "entity.hpp"
//#include "object.hpp"
//#include "properties.hpp"
//#include "alignment.hpp"
//
//using namespace std;
//using boost::add_vertex;
//using boost::vertices;
//using boost::tie;
//
//auto detect_objects(TaskUnderstanding& task) -> void
//{
//    const auto clouds = detect_objects(task.before_scene.cloud);
//    
//    /* Segment out objects on before_scene */
//    for (int i = 0; i < clouds.size(); ++i) {
//        auto& c = clouds[i];
//        auto props = detect_properties(c);
//        auto oid = "object_" + to_string(i);
//        Entity entity{Entity::Type::object, oid, props.position, props.rotation, c};
//        add_vertex(entity, task.before_scene.graph);
//    }
//    
//    /* Find the same objects in after_scene */
//    SceneGraph::vertex_iterator i, end;
//    auto& graph = task.before_scene.graph;
//    auto& after_cloud = task.after_scene.cloud;
//    
//    for (tie(i, end) = vertices(graph); i != end; ++i) {
//        auto e = graph[*i];
//        
//        if (e.type != Entity::Type::object) {
//            continue;
//        }
//        
//        shared_ptr<PointCloud> aligned_cloud;
//        Transform transform;
//        tie(transform, aligned_cloud) = align(e.cloud, after_cloud);
//        auto props = detect_properties(aligned_cloud);
//        Entity entity{Entity::Type::object, e.id, props.position, props.rotation, aligned_cloud};
//        add_vertex(entity, task.after_scene.graph);
//        task.object_transforms.push_back(transform);
//    }
//}
//
//auto detect_change(TaskUnderstanding& task) -> void
//{
//    task.focus_descriptors.push_back(0);
//    task.focus_descriptors.push_back(1);
//}
//
//auto detect_features(TaskUnderstanding& task) -> void
//{
//    
//    
//    /* Mass center */
//    const auto props = detect_properties(object.cloud);
//    add_vertex(Entity{Entity::Type::mass_center, props.mass_center}, )
//    object.features.push_back(make_shared<MassCenterFeature>("mass_center", props.mass_center));
//    
//    /* Holes */
//    const auto holes = find_holes(object.cloud);
//    for (int i = 0; i < holes.size(); ++i) {
//        auto& h = holes[i];
//        auto props = detect_properties(h);
//        auto hid = "hole_" + to_string(i);
//        object.features.push_back(make_shared<HoleFeature>(hid, h, props.position, props.rotation));
//    }
//    
//    /* Pegs */
//    const auto pegs = detect_pegs(object.cloud);
//    for (int i = 0; i < pegs.size(); ++i) {
//        auto& p = pegs[i];
//        auto props = detect_properties(p);
//        auto pid = "peg_" + to_string(i);
//        object.features.push_back(make_shared<PegFeature>(pid, p, props.position, props.rotation));
//    }
//}
