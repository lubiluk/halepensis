#include "io.hpp"
#include "filtering.hpp"
#include "normals.hpp"
#include "regions.hpp"
#include "visualization.hpp"
#include "ransac.hpp"
#include "search.hpp"
#include "features.hpp"
#include "clusters.hpp"
#include <iostream>
#include <algorithm>
#include <vector>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#pragma clang diagnostic pop

auto find_apertures(std::vector<std::shared_ptr<point_cloud>> clusters, std::vector<std::shared_ptr<surface_normals>> cluster_normals)
-> std::vector<std::shared_ptr<point_cloud>>
{
    std::vector<std::shared_ptr<point_cloud>> results;

    for (int i = 0; i < clusters.size(); ++i)
    {
        const auto& cloud = clusters[i];
        const auto& normals = cluster_normals[i];
        
        const auto inliers = estimate_boundaries(cloud, normals);
        const auto bound_cloud = extract_cloud(cloud, inliers);

        results.push_back(bound_cloud);
    }

    return results;
}

int main(int argc, const char *argv[])
{
    auto cloud = load_cloud("/Users/lubiluk/Code/halepensis/data/cutting_board_scan/cutting_board_scan_6.pcd");
    
    if (!cloud)
    {
        std::cout << "Could not open file" << std::endl;
        return -1;
    }
    
    cloud = downsample(cloud);
    cloud = filter_depth(cloud);
    const auto normals = compute_normals(cloud);
    const auto indices = segment_regions(cloud, normals);
    
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
    
    const auto circles = find_apertures(clusters, cluster_normals);
    view_clusters(cloud, circles);
    
    
    //
    
    const auto inliers = estimate_boundaries(cloud, normals);
    const auto bound_cloud = extract_cloud(cloud, inliers);
    const auto bound_normals = extract_normals(normals, inliers);
    const auto bound_cluster_indices = segment_regions(bound_cloud, bound_normals);
    std::vector<std::shared_ptr<point_cloud>> bound_clusters;
    std::transform(bound_cluster_indices.begin(), bound_cluster_indices.end(), std::back_inserter(bound_clusters), [&bound_cloud](const auto& i) -> auto
    {
        return extract_cloud(bound_cloud, i);
    });
    std::vector<std::shared_ptr<point_cloud>> holes;
    for (const auto& c: bound_clusters)
    {
        float x { 0.0 };
        float y { 0.0 };
        float z { 0.0 };
        
        for (const auto& p: *c)
        {
            x += p.x;
            y += p.y;
            z += p.z;
        }
        
        x = x / c->size();
        y = y / c->size();
        z = z / c->size();
        
        point center {x, y, z};
        
        if (count_nearby_points(cloud, center, 0.005) == 0)
        {
            holes.push_back(c);
        }
    }
    
    view_clusters(cloud, holes);

    return 0;
}
