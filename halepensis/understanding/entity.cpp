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
               std::shared_ptr<PointCloud> cloud,
               Point min_corner,
               Point max_corner):
type(type),
id(id),
position(position),
orientation(orientation),
cloud(cloud),
min_corner(min_corner),
max_corner(max_corner)
{
    
}

Entity::Entity():
type(Type::object),
id(""),
position(Vector{}),
orientation(Quaternion{}),
cloud(nullptr),
min_corner({}),
max_corner({})
{
    
}

auto Entity::transformed(const Transform& transform) const -> Entity
{
    Affine3f t{transform};
    auto t_position = t * position;
    auto t_orientation = t * orientation;
    auto t_cloud = make_shared<PointCloud>();
    auto t_min_corn = t * Vector{min_corner.x, min_corner.y, min_corner.z};
    auto t_max_corn = t * Vector{max_corner.x, max_corner.y, max_corner.z};
    
    if (cloud) {
        transformPointCloud(*cloud, *t_cloud, transform);
    }
    
    return {
        type,
        id,
        t_position,
        t_orientation,
        t_cloud,
        Point{t_min_corn[0], t_min_corn[1], t_min_corn[2]},
        Point{t_max_corn[0], t_max_corn[1], t_max_corn[2]}
    };
}
