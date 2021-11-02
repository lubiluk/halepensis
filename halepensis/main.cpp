#include "io.hpp"
#include "filtering.hpp"
#include "normals.hpp"
#include "ransac.hpp"
#include "visualization.hpp"
#include "clusters.hpp"
#include "alignment.hpp"
#include "hole.hpp"
#include "scene_graph.hpp"
#include "scene_visualization.hpp"
#include "pipeline.hpp"

#include <iostream>
#include <algorithm>
#include <vector>

/*  */
/*  */

int main(int argc, const char *argv[])
{
    auto cloud_before = load_cloud(argv[1]);
    if (!cloud_before)
    {
        std::cout << "Could not open file: " << argv[1] << std::endl;
        return -1;
    }
    
    auto cloud_after = load_cloud(argv[2]);
    if (!cloud_after)
    {
        std::cout << "Could not open file: " << argv[1] << std::endl;
        return -1;
    }
    
    cloud_before = downsample(cloud_before);
    cloud_after = downsample(cloud_after);

    compute_normals(cloud_before);
    compute_normals(cloud_after);

    /* Remove walls */
    auto indics = fit_plane(cloud_before);
    cloud_before = extract_cloud(cloud_before, std::get<0>(indics.value()), true);
    indics = fit_plane(cloud_after);
    cloud_after = extract_cloud(cloud_after, std::get<0>(indics.value()), true);

    view(cloud_before, cloud_after);
    
    /* Task Reasoning Part */
    
    TaskUnderstanding task { cloud_before, cloud_after };
    detect_objects(task);
    view_clusters(task.before_scene.cloud, task.before_scene.object_clouds());
    detect_change(task);
    
    for (const auto& i : task.focus_indices)
    {
        detect_features(task.before_scene.objects[i]);
        copy_features(task.before_scene.objects[i], task.after_scene.objects[i], task.object_transforms[i]);
    }
    
    view_scenes(task);
    

    /* segment out objects */
    auto indices = extract_euclidean_clusters(cloud_before, 0.05, 100, 10000);

    std::vector<std::shared_ptr<point_cloud>> objects;
    std::transform(indices.begin(), indices.end(), std::back_inserter(objects), [&cloud_before](const auto& i) -> auto
    {
        return extract_cloud(cloud_before, i);
    });

    view_clusters(cloud_before, objects);

    auto hanger = objects.front();
    const auto hanger_alignment = align(hanger, cloud_after);
    view_clusters(cloud_after, std::vector<std::shared_ptr<point_cloud>> { hanger_alignment.second });

    auto hooks = objects[1];
    view(hooks);

    /* Find holes */
    const auto holes = find_holes(hanger);
    view_clusters(hanger, holes);

    
    return 0;
}
