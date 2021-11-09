#include "peg.hpp"
#include "ransac.hpp"
#include "clusters.hpp"
#include "filtering.hpp"

auto detect_pegs(const std::shared_ptr<PointCloud> &cloud)
-> std::vector<std::shared_ptr<PointCloud>>
{
    auto sphere = fit_sphere(cloud);
    
    if (!sphere) {
        return {};
    }
    
    auto indices = std::get<0>(sphere.get());
    auto all_pegs = extract_cloud(cloud, indices, true);
    auto peg_indices = extract_euclidean_clusters(all_pegs, 0.02, 10, 500);
    
    std::vector<std::shared_ptr<point_cloud>> pegs;
    std::transform(peg_indices.begin(), peg_indices.end(),
                   std::back_inserter(pegs),
                   [&cloud](const auto& i) -> auto {
        return extract_cloud(cloud, i);
    });
    
    return pegs;
}
