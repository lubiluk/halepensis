#include "scene_entity.hpp"

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

scene_entity::scene_entity(entity_type type,
               entity_id id,
               vec3 position,
               rot_mat orientation,
               std::shared_ptr<point_cloud> cloud,
               point min_corner,
               point max_corner):
type(type),
id(id),
position(position),
orientation(orientation),
cloud(cloud),
min_corner(min_corner),
max_corner(max_corner)
{
    
}

scene_entity::scene_entity():
type(entity_type::object),
id(""),
position(vec3{}),
orientation(quat{}),
cloud(nullptr),
min_corner({}),
max_corner({})
{
    
}

auto scene_entity::transformed(const mat44& transform) const -> scene_entity
{
    Affine3f t{transform};
    auto t_position = t * position;
    auto t_orientation = t * orientation;
    auto t_cloud = make_shared<point_cloud>();
    auto t_min_corn = t * vec3{min_corner.x, min_corner.y, min_corner.z};
    auto t_max_corn = t * vec3{max_corner.x, max_corner.y, max_corner.z};
    
    if (cloud) {
        transformPointCloud(*cloud, *t_cloud, transform);
    }
    
    return {
        type,
        id,
        t_position,
        t_orientation,
        t_cloud,
        point{t_min_corn[0], t_min_corn[1], t_min_corn[2]},
        point{t_max_corn[0], t_max_corn[1], t_max_corn[2]}
    };
}
