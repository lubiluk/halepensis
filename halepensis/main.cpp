#include "io.hpp"
#include "filtering.hpp"
#include "normals.hpp"
#include "ransac.hpp"
#include "visualization.hpp"
#include "task_understanding.hpp"
#include "scene_visualization.hpp"
#include "graph_visualization.hpp"

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
    
    view(cloud_before, cloud_after);
    
    cloud_before = remove_outliers(cloud_before);
    cloud_after = remove_outliers(cloud_after);
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
    
    task_understanding task { cloud_before, cloud_after };
    task.detect_objects();
    view_clusters(task.before_scene.cloud, task.before_scene.object_clouds());
    task.detect_change();
    task.detect_features();
    task.describe_relations();

    view_scenes(task);
    // There is a bug that prevents us from showing graphs side by side...
    view(task.before_scene.graph);
    view(task.after_scene.graph);

    task.describe_task();
    view(task.task_description);
    
    /* Tests */
    
    
    return 0;
}
