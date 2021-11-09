#include "scene_understanding.hpp"

SceneUnderstanding::SceneUnderstanding(const std::shared_ptr<PointCloud> cloud):
cloud(cloud)
{
    
}

auto SceneUnderstanding::object_clouds() -> std::vector<std::shared_ptr<PointCloud>>
{
    using namespace std;
    vector<shared_ptr<PointCloud>> clouds;
    transform(objects.begin(), objects.end(), back_inserter(clouds),
              [](const auto& o) -> auto { return o.cloud; });
    
    return clouds;
}
