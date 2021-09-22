#include "io.hpp"
#include "filtering.hpp"
#include "normals.hpp"
#include "regions.hpp"
#include "visualization.hpp"
#include "ransac.hpp"
#include "search.hpp"
#include "features.hpp"
#include "clusters.hpp"
#include "scaling.hpp"
#include "hole.hpp"
#include "outliers.hpp"
#include <iostream>
#include <algorithm>
#include <vector>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#pragma clang diagnostic pop

int main(int argc, const char *argv[])
{
    auto cloud = load_cloud("/Users/lubiluk/Code/halepensis/data/hooks_scan/hooks_scan_2.pcd");
    
    if (!cloud)
    {
        std::cout << "Could not open file" << std::endl;
        return -1;
    }
    
    cloud = scale(cloud, 0.001f);
    cloud = downsample(cloud);
    cloud = filter_depth(cloud, 1.0f);
    
    view(cloud);
    
    const auto normals = compute_normals(cloud);
    
    const auto indics = fit_plane(cloud);
    cloud = extract_cloud(cloud, std::get<0>(indics.value()), true);
    view(cloud);
    
    const auto indices = extract_euclidean_clusters(cloud, 0.02, 10, 10000);

    std::vector<std::shared_ptr<point_cloud>> clusters;
    std::transform(indices.begin(), indices.end(), std::back_inserter(clusters), [&cloud](const auto& i) -> auto
    {
        return extract_cloud(cloud, i);
    });
    std::vector<std::shared_ptr<surface_normals>> cluster_normals;
    std::transform(indices.begin(), indices.end(), std::back_inserter(cluster_normals), [&normals](const auto& i) -> auto
    {
        return extract_normals(normals, i);
    });

    view_clusters(cloud, clusters);
//
//    const auto holes = find_holes(cloud, normals);
//    view_clusters(cloud, holes);

    return 0;
}
