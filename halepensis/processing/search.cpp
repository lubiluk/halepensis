#include "search.hpp"
#include "filtering.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#pragma clang diagnostic pop

auto count_nearby_points(const std::shared_ptr<point_cloud>& input_cloud,
                         const point search_point, const float radius) -> int
{
    pcl::KdTreeFLANN<point> kdtree;
    kdtree.setInputCloud(input_cloud);
    
    std::vector<int> point_idx_radius_search;
    std::vector<float> point_radius_squared_distance;
    
    return kdtree.radiusSearch(search_point, radius, point_idx_radius_search, point_radius_squared_distance);
}
