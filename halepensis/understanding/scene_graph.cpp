#include "scene_graph.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#pragma clang diagnostic pop

SceneEntity::SceneEntity(std::string id, Vector position, Quaternion orientation):
id(id),
position(position),
orientation(orientation)
{
    
}

SceneEntity::~SceneEntity()
{
    
}

auto SceneEntity::transform(const Transform& transform) -> void
{
    Eigen::Affine3f t{transform};
    position = t * position;
    orientation = t.rotate(orientation).rotation();
}

CloudEntity::CloudEntity(std::string id,
                         std::shared_ptr<PointCloud> cloud,
                         Vector position,
                         Quaternion orientation):
SceneEntity(id, position, orientation),
cloud(cloud)
{
    
}

CloudEntity::CloudEntity(const CloudEntity& original):
SceneEntity(original),
cloud(std::make_shared<PointCloud>())
{
    pcl::copyPointCloud(*(original.cloud), *cloud);
}

CloudEntity::~CloudEntity()
{
    
}

auto CloudEntity::transform(const Transform& transform) -> void
{
    SceneEntity::transform(transform);
    pcl::transformPointCloud(*cloud, *cloud, transform);
}

HoleFeature::HoleFeature(std::string id,
                         std::shared_ptr<PointCloud> cloud,
                         Vector position,
                         Quaternion orientation):
CloudEntity(id, cloud, position, orientation)
{
    
}

HoleFeature::HoleFeature(const HoleFeature& original):
CloudEntity(original)
{
    
}

auto HoleFeature::clone() -> std::shared_ptr<SceneEntity> const
{
    return std::make_shared<HoleFeature>(*this);
}

MassCenter::MassCenter(std::string id, Vector position):
SceneEntity(id, position)
{
    
}

auto MassCenter::clone() -> std::shared_ptr<SceneEntity> const
{
    return std::make_shared<MassCenter>(*this);
}

SceneObject::SceneObject(std::string id,
                         std::shared_ptr<PointCloud> cloud,
                         Vector position,
                         Quaternion orientation):
CloudEntity(id, cloud, position, orientation)
{
    
}

SceneObject::SceneObject(const SceneObject& original):
CloudEntity(original)
{
    
}

auto SceneObject::clone() -> std::shared_ptr<SceneEntity> const
{
    return std::make_shared<SceneObject>(*this);
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
