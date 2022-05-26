#include "peg.hpp"
#include "ransac.hpp"
#include "regions.hpp"
#include "filtering.hpp"

#include <chrono>

auto detect_pegs(const std::shared_ptr<point_cloud>& cloud)
-> std::vector<std::shared_ptr<point_cloud>>
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    
    auto peg_indices = segment_regions(cloud);
    
    std::vector<std::shared_ptr<point_cloud>> pegs;
    std::transform(peg_indices.begin(), peg_indices.end(),
                   std::back_inserter(pegs),
                   [&cloud](const auto& i) -> auto {
        return extract_cloud(cloud, i);
    });
    
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "detect_pegs time = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
    
    return pegs;
}
