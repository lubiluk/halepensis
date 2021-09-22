#include "scaling.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#pragma clang diagnostic pop


auto scale(const std::shared_ptr<point_cloud>& cloud, const float factor)
-> std::shared_ptr<point_cloud>
{
    auto output = std::make_shared<point_cloud>(*cloud);
    
    for (auto& point : *output)
    {
        point.x /= 1000.0f;
        point.y /= 1000.0f;
        point.z /= 1000.0f;
    }
    
    return output;
}
