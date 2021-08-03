#include "io.hpp"
#include "filtering.hpp"
#include "normals.hpp"
#include "regions.hpp"
#include "visualization.hpp"
#include <iostream>
#include <algorithm>
#include <vector>

//auto scene_entity::find_apertures() -> std::vector<const entity_surface>
//{
//    std::vector<const entity_surface> results;
//
//    const auto regions = surface.downsampled().without_background().with_normals().find_regions();
//
//    for (const auto& r: regions)
//    {
//        const auto circle = fit_model(1, r.cloud, r.normals);
//
//        if (!circle) continue;
//
//        // Check if there's a hole
//        const auto [ inliers, coefficients ] = *circle;
//        const auto circle_cloud = extract_cloud(surface.cloud, inliers);
//        const point search_point(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
//
//        if (count_nearby_points(circle_cloud, search_point, 0.02) == 0 )
//        {
//            results.push_back(entity_surface(circle_cloud));
//        }
//    }
//
//    return results;
//}

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
       
    view_clusters(cloud, clusters);

    return 0;
}
