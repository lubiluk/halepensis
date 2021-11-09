#include "task_understanding.hpp"

TaskUnderstanding::TaskUnderstanding(const std::shared_ptr<PointCloud> before_cloud, const std::shared_ptr<PointCloud> after_cloud):
before_scene(before_cloud),
after_scene(after_cloud)
{
    
}
