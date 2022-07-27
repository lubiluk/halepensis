#include "surface.hpp"
#include "ransac.hpp"
#include "regions.hpp"
#include "filtering.hpp"

#include <chrono>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/common/io.h>
#pragma clang diagnostic pop

using namespace std;

auto detect_surfaces(const std::shared_ptr<point_cloud> &cloud)
-> std::vector<std::shared_ptr<point_cloud>>
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    
    auto c = make_shared<point_cloud>();
    pcl::copyPointCloud(*cloud, *c);
    
    std::vector<std::shared_ptr<point_cloud>> surfaces;
    
    do {
        auto indics = fit_plane(c, 0.003);
        auto surface = extract_cloud(c, std::get<0>(indics.value()), false);
        c = extract_cloud(c, std::get<0>(indics.value()), true);
        
        if (surface->size() < 500) continue;
        
        surfaces.push_back(surface);
    } while (c->size() > 0.4 * cloud -> size());
    
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "detect_surfaces time = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
    
    return surfaces;
}

