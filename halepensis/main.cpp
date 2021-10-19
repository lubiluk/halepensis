#include "io.hpp"
#include "filtering.hpp"
#include "normals.hpp"
#include "ransac.hpp"
#include "visualization.hpp"
#include "clusters.hpp"
#include "alignment.hpp"
#include "hole.hpp"
#include "scene_graph.hpp"
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
    
    cloud_before = downsample(cloud_before);
    cloud_after = downsample(cloud_after);

    compute_normals(cloud_before);
    compute_normals(cloud_after);

    // Remove walls
    auto indics = fit_plane(cloud_before);
    cloud_before = extract_cloud(cloud_before, std::get<0>(indics.value()), true);
    indics = fit_plane(cloud_after);
    cloud_after = extract_cloud(cloud_after, std::get<0>(indics.value()), true);

    view(cloud_before, cloud_after);

    // segment out objects
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
    
//    auto hooks = objects.back();
//    const auto hooks_alignment = align(hooks, cloud_after);
//    view_clusters(cloud_after, std::vector<std::shared_ptr<point_cloud>>{ hooks_alignment.second });
    
    // Find holes
    const auto holes = find_holes(hanger);
    view_clusters(hanger, holes);
    
    
    SceneGraph graph;
    Object hanger_obj { "hanger" };
    Hole hole;
    hanger_obj.features.push_back(hole);
    graph.objects.push_back(hanger_obj);
    
    view(graph);
    
    return 0;
}
