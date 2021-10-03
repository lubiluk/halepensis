#include "io.hpp"
#include "filtering.hpp"
#include "normals.hpp"
#include "ransac.hpp"
#include "visualization.hpp"
#include "clusters.hpp"
#include "alignment.hpp"

#include <iostream>
#include <algorithm>
#include <vector>


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
    auto indices = extract_euclidean_clusters(cloud_before, 0.02, 100, 10000);
    indices.erase(indices.begin());
    indices.erase(std::prev(indices.end()));
    
    std::vector<std::shared_ptr<point_cloud>> objects;
    std::transform(indices.begin(), indices.end(), std::back_inserter(objects), [&cloud_before](const auto& i) -> auto
    {
        return extract_cloud(cloud_before, i);
    });
    
    view_clusters(cloud_before, objects);
    
    auto hanger = objects.front();
    // Remove hand
    hanger = filter_field(hanger, "x", -0.1, 0.5);
    
    view(hanger);

    const auto alignment = align(hanger, cloud_after);
    std::vector<std::shared_ptr<point_cloud>> vec{ alignment.second };
    view_clusters(cloud_after, vec);
    
//    const auto indics = fit_plane(cloud);
//    cloud = extract_cloud(cloud, std::get<0>(indics.value()), true);
//    view(cloud);
//
//    const auto indices = extract_euclidean_clusters(cloud, 0.02, 100, 10000);
//
//    std::vector<std::shared_ptr<point_cloud>> clusters;
//    std::transform(indices.begin(), indices.end(), std::back_inserter(clusters), [&cloud](const auto& i) -> auto
//    {
//        return extract_cloud(cloud, i);
//    });
//    std::vector<std::shared_ptr<surface_normals>> cluster_normals;
//    std::transform(indices.begin(), indices.end(), std::back_inserter(cluster_normals), [&normals](const auto& i) -> auto
//    {
//        return extract_normals(normals, i);
//    });
//    
//    const std::vector<std::shared_ptr<point_cloud>> objects( clusters.begin(), clusters.begin() + 2);
//
//    view_clusters(cloud, objects);
//
//    const auto holes = find_holes(cloud, normals);
//    view_clusters(cloud, holes);

    return 0;
}
