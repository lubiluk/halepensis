#include "scene_graph.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#pragma clang diagnostic pop







TaskUnderstanding::TaskUnderstanding(const std::shared_ptr<PointCloud> before_cloud, const std::shared_ptr<PointCloud> after_cloud):
before_scene(before_cloud),
after_scene(after_cloud)
{
    
}
