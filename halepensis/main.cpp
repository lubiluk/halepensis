#include "io.hpp"
#include "filtering.hpp"
#include "normals.hpp"
#include "ransac.hpp"
#include "visualization.hpp"
#include "task_understanding.hpp"
#include "scene_visualization.hpp"
#include "understanding_visualization.hpp"

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
    
    /* Graph */

    view(cloud_before, cloud_after);
    
    /* Task Reasoning Part */
    
    TaskUnderstanding task { cloud_before, cloud_after };
    task.detect_objects();
    view_clusters(task.before_scene.cloud, task.before_scene.object_clouds());
    task.detect_change();
    task.detect_features();
    task.describe_relations();
    
    view_scenes(task);
    // There is a bug that prevents us to show graphs side by side...
    view(task.before_scene);
    view(task.after_scene);
    

    /* Tests */
    
    
//    /* segment out objects */
//    auto indices = extract_euclidean_clusters(cloud_before, 0.05, 100, 10000);
//
//    std::vector<std::shared_ptr<point_cloud>> objects;
//    std::transform(indices.begin(), indices.end(), std::back_inserter(objects), [&cloud_before](const auto& i) -> auto
//    {
//        return extract_cloud(cloud_before, i);
//    });
//
//    auto hooks = objects[1];
//    view(hooks);
//
//    auto pegs = detect_pegs(hooks);
//    view_clusters(hooks, pegs);
    
    return 0;
}
