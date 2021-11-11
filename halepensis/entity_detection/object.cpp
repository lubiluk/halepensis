#include "object.hpp"
#include "clusters.hpp"
#include "ransac.hpp"
#include "filtering.hpp"

auto detect_objects(const std::shared_ptr<point_cloud> &cloud, const int n)
-> std::vector<std::shared_ptr<point_cloud>>
{
    const auto indices = extract_euclidean_clusters(cloud, 0.05, 100, 10000);

    std::vector<std::shared_ptr<point_cloud>> clusters;
    std::transform(indices.begin(), indices.end(), std::back_inserter(clusters), [&cloud](const auto& i) -> auto
    {
        return extract_cloud(cloud, i);
    });
    
    const std::vector<std::shared_ptr<point_cloud>> objects( clusters.begin(), clusters.begin() + n);
    
    return objects;
}
