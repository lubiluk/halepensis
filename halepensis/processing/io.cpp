#include "io.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#pragma clang diagnostic pop

auto load_cloud(const std::string& pcd_file) -> std::shared_ptr<point_cloud>
{
    auto cloud = std::make_shared<point_cloud>();
    
    if (pcl::io::loadPCDFile<point>(pcd_file, *cloud) == -1)
    {
        return nullptr;
    }
    
    return cloud;
}
