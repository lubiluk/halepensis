#include "hole.hpp"
#include "features.hpp"
#include "filtering.hpp"
#include "regions.hpp"
#include "search.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#pragma clang diagnostic pop

auto find_holes(const std::shared_ptr<point_cloud> &cloud)
-> std::vector<std::shared_ptr<point_cloud>>
{
    const auto inliers = estimate_boundaries(cloud);
    const auto bound_cloud = extract_cloud(cloud, inliers);
    const auto bound_cluster_indices = segment_regions(bound_cloud);
    std::vector<std::shared_ptr<point_cloud>> bound_clusters;
    std::transform(bound_cluster_indices.begin(),
                   bound_cluster_indices.end(),
                   std::back_inserter(bound_clusters),
                   [&bound_cloud](const auto& i) -> auto {
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
    
    return holes;
}
