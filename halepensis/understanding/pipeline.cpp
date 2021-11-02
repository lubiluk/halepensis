#include "pipeline.hpp"

#include "scene_graph.hpp"
#include "object.hpp"
#include "alignment.hpp"
#include "hole.hpp"
#include "properties.hpp"

auto detect_objects(TaskUnderstanding& task) -> void
{
    using namespace std;
    
    const auto clouds = find_objects(task.before_scene.cloud);
    
    /* Segment out objects on before_scene */
    for (int i = 0; i < clouds.size(); ++i) {
        auto& c = clouds[i];
        auto props = detect_properties(c);
        auto oid = "object_" + to_string(i);
        task.before_scene.objects.emplace_back(oid, c, props.position, props.rotation);
    }
    
    /* Find the same objects in after_scene */
    for (const auto& o : task.before_scene.objects)
    {
        auto alignment = align(o.cloud, task.after_scene.cloud);
        auto aligned_cloud = alignment.second;
        auto props = detect_properties(aligned_cloud);
        auto ao = SceneObject{o.id, aligned_cloud, props.position, props.rotation};
        task.after_scene.objects.push_back(ao);
        task.object_transforms.push_back(alignment.first);
    }
}

auto detect_change(TaskUnderstanding& task) -> void
{
    task.focus_indices.push_back(0);
    task.focus_indices.push_back(1);
}

auto detect_features(SceneObject& object) -> void
{
    using namespace std;
    
    /* Mass center */
    const auto props = detect_properties(object.cloud);
    object.features.push_back(make_shared<MassCenter>("mass_center", props.mass_center));
    
    /* Find holes */
    const auto holes = find_holes(object.cloud);
    
    for (int i = 0; i < holes.size(); ++i) {
        auto& h = holes[i];
        const auto props = detect_properties(h);
        auto hid = "hole_" + to_string(i);
        object.features.push_back(make_shared<HoleFeature>(hid, h, props.position, props.rotation));
    }
}


auto copy_features(const SceneObject& src, SceneObject& dst, Transform& transform) -> void
{
    std::transform(src.features.begin(), src.features.end(), back_inserter(dst.features),
              [&transform](const auto& f) -> auto {
        auto copy = f->clone();
        copy->transform(transform);
        return copy;
    });
}
