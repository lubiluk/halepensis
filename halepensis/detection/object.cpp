#include "object.hpp"
#include "clusters.hpp"
#include "ransac.hpp"
#include "filtering.hpp"

auto find_objects(const std::shared_ptr<point_cloud> &cloud, const std::shared_ptr<surface_normals> &normals, const int n)
-> std::vector<std::shared_ptr<point_cloud>>
{
    const auto indices = extract_euclidean_clusters(cloud, 0.02, 100, 10000);

    std::vector<std::shared_ptr<point_cloud>> clusters;
    std::transform(indices.begin(), indices.end(), std::back_inserter(clusters), [&cloud](const auto& i) -> auto
    {
        return extract_cloud(cloud, i);
    });
    
    const std::vector<std::shared_ptr<point_cloud>> objects( clusters.begin(), clusters.begin() + n);
    
    return objects;
}


//auto find_object(const std::shared_ptr<point_cloud> &cloud, const std::shared_ptr<point_cloud> &scene)
//-> std::vector<std::shared_ptr<point_cloud>>
//{
//    
//}
