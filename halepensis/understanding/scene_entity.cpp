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
using boost::optional;
using boost::none;

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

auto scene_entity::type_description() const -> string
{
    return entity_type_to_string(type);
}


auto entity_type_to_string(entity_type type) -> string
{
    switch (type) {
        case entity_type::object:
            return "object";
        case entity_type::mass_center:
            return "mass_center";
        case entity_type::hole:
            return "hole";
        case entity_type::peg:
            return "peg";
        case entity_type::surface:
            return "surface";
        case entity_type::edge:
            return "edge";
        case entity_type::base:
            return "base";
        default:
            return "unknown_entity_type";
    }
}

auto entity_type_from_string(const string& string) -> optional<entity_type>
{
    if (string == entity_type_to_string(entity_type::object)) {
        return entity_type::object;
    }
    
    if (string == entity_type_to_string(entity_type::mass_center)) {
        return entity_type::mass_center;
    }
    
    if (string == entity_type_to_string(entity_type::hole)) {
        return entity_type::hole;
    }
    
    if (string == entity_type_to_string(entity_type::peg)) {
        return entity_type::peg;
    }
    
    if (string == entity_type_to_string(entity_type::surface)) {
        return entity_type::surface;
    }
    
    if (string == entity_type_to_string(entity_type::edge)) {
        return entity_type::edge;
    }
    
    if (string == entity_type_to_string(entity_type::base)) {
        return entity_type::base;
    }
    
    return none;
}

