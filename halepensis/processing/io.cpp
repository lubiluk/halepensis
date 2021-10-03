#include "io.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#pragma clang diagnostic pop

auto load_cloud(const std::string& pcd_file) -> std::shared_ptr<point_cloud>
{
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_file, *cloud) == -1)
    {
        return nullptr;
    }
    
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    const auto result = std::make_shared<point_cloud>();
    pcl::copyPointCloud(*cloud, *result);
    
    return result;
}
