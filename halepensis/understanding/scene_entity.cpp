#include "scene_entity.hpp"

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
