#include "entity.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#pragma clang diagnostic pop

using std::string;
using std::shared_ptr;
using std::make_shared;
using pcl::copyPointCloud;
using Eigen::Affine3f;

Entity::Entity(Type type,
               EntityId id,
               Vector position,
               Rotation orientation,
               std::shared_ptr<PointCloud> cloud):
type(type),
id(id),
position(position),
orientation(orientation),
cloud(cloud)
{
    
}

Entity::Entity():
type(Type::object),
id(""),
position(Vector{}),
orientation(Rotation{}),
cloud(nullptr)
{
    
}

auto Entity::transformed(const Transform& transform) const -> Entity
{
    Affine3f t{transform};
    auto t_position = t * position;
    auto t_orientation = t * orientation;
    auto t_cloud = make_shared<PointCloud>();
    
    if (cloud) {
        transformPointCloud(*cloud, *t_cloud, transform);
    }
    
    return {
        type,
        id,
        t_position,
        t_orientation,
        t_cloud
    };
}
