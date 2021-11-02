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
    transform(clouds.begin(), clouds.end(), back_inserter(task.before_scene.objects),
              [](const auto& c) -> SceneObject {
        const auto props = detect_properties(c);
        return {c, props.position, props.rotation};
    });
    
    /* Find the same objects in after_scene */
    for (const auto& o : task.before_scene.objects)
    {
        const auto aligned_cloud = align(o.cloud, task.after_scene.cloud).second;
        const auto props = detect_properties(aligned_cloud);
        const auto ao = SceneObject{aligned_cloud, props.position, props.rotation};
        task.after_scene.objects.push_back(ao);
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
    object.features.push_back(make_shared<MassCenter>(props.mass_center));
    
    /* Find holes */
    const auto holes = find_holes(object.cloud);
    
    for (const auto& h : holes) {
        const auto props = detect_properties(h);
        object.features.push_back(make_shared<HoleFeature>(h, props.position, props.rotation));
    }
}

auto copy_features(const SceneObject& src, SceneObject& dst) -> void
{
//    using namespace std;
//    transform(src.features.begin(), src.features.end(), back_inserter(dst.features),
//              [](const auto& f) -> auto { return f; });
}
