#include "scene_graph.hpp"
#include "id_provider.hpp"


SceneEntity::SceneEntity(std::string id, const Vector position, const Quaternion orientation):
id(id),
position(position),
orientation(orientation)
{
    
}

SceneEntity::~SceneEntity()
{
    
}

CloudEntity::CloudEntity(const std::string id,
                         const std::shared_ptr<PointCloud> cloud,
                         const Vector position,
                         const Quaternion orientation):
SceneEntity(id, position, orientation),
cloud(cloud)
{
    
}

CloudEntity::~CloudEntity()
{
    
}

HoleFeature::HoleFeature(const std::shared_ptr<PointCloud> cloud,
                         const Vector position,
                         const Quaternion orientation):
CloudEntity(IdProvider::instance().next_id("hole"), cloud, position, orientation)
{
    
}

MassCenter::MassCenter(const Vector position):
SceneEntity(IdProvider::instance().next_id("mass_center"), position)
{
    
}

SceneObject::SceneObject(const std::shared_ptr<PointCloud> cloud,
               const Vector position,
               const Quaternion orientation):
CloudEntity(IdProvider::instance().next_id("object"), cloud, position, orientation)
{
    
}

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

TaskUnderstanding::TaskUnderstanding(const std::shared_ptr<PointCloud> before_cloud, const std::shared_ptr<PointCloud> after_cloud):
before_scene(before_cloud),
after_scene(after_cloud)
{
    
}
