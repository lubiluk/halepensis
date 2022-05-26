#include "object.hpp"
#include "clusters.hpp"
#include "ransac.hpp"
#include "filtering.hpp"

#include <chrono>

auto detect_objects(const std::shared_ptr<point_cloud> &cloud, const int n)
-> std::vector<std::shared_ptr<point_cloud>>
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    
    const auto indices = extract_euclidean_clusters(cloud, 0.03, 100, 10000);

    std::vector<std::shared_ptr<point_cloud>> clusters;
    std::transform(indices.begin(), indices.end(), std::back_inserter(clusters), [&cloud](const auto& i) -> auto
    {
        return extract_cloud(cloud, i);
    });
    
    const std::vector<std::shared_ptr<point_cloud>> objects( clusters.begin(), clusters.begin() + n);
    
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    
    std::cout << "detect_objects time = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
    
    return objects;
}
