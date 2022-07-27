#include "relation_detection.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#pragma clang diagnostic pop

using namespace boost::geometry;
using model::d3::point_xyz;
using model::box;

const float epsilon_vertical = 0.01;
const float epsilon_horizontal = 0.025;
const float touching_epsilon = 0.01;

auto is_below(const scene_entity& entity1, const scene_entity& entity2) -> bool
{
    return entity1.position.y() < entity2.position.y() - epsilon_vertical
    && entity1.position.x() < entity2.position.x() + epsilon_horizontal
    && entity1.position.x() > entity2.position.x() - epsilon_horizontal
    && entity1.position.z() < entity2.position.z() + epsilon_horizontal
    && entity1.position.z() > entity2.position.z() - epsilon_horizontal;
}

auto is_inside(const scene_entity& entity1, const scene_entity& entity2) -> bool
{
    box<point_xyz<float>> box1{{entity1.min_corner.x, entity1.min_corner.y, entity1.min_corner.z},
        {entity1.max_corner.x, entity1.max_corner.y, entity1.max_corner.z}};
    box<point_xyz<float>> box2{{entity2.min_corner.x, entity2.min_corner.y, entity2.min_corner.z},
        {entity2.max_corner.x, entity2.max_corner.y, entity2.max_corner.z}};
    
    auto o = overlaps(box1, box2);
    auto d1 = abs(box1.max_corner().x() - box1.min_corner().x())
    + abs(box1.max_corner().y() - box1.min_corner().y())
    + abs(box1.max_corner().z() - box1.min_corner().z());
    auto d2 = abs(box2.max_corner().x() - box2.min_corner().x())
    + abs(box2.max_corner().y() - box2.min_corner().y())
    + abs(box2.max_corner().z() - box2.min_corner().z());
    
    return o && d2 < d1;
}

auto is_touching(const scene_entity& entity1, const scene_entity& entity2) -> bool
{
    box<point_xyz<float>> box1 {
        {
            entity1.min_corner.x - touching_epsilon,
            entity1.min_corner.y - touching_epsilon,
            entity1.min_corner.z - touching_epsilon
            
        },
        {
            entity1.max_corner.x + touching_epsilon,
            entity1.max_corner.y + touching_epsilon,
            entity1.max_corner.z + touching_epsilon
            
        }
    };
    box<point_xyz<float>> box2 {
        {
            entity2.min_corner.x - touching_epsilon,
            entity2.min_corner.y - touching_epsilon,
            entity2.min_corner.z - touching_epsilon
            
        },
        {
            entity2.max_corner.x + touching_epsilon,
            entity2.max_corner.y + touching_epsilon,
            entity2.max_corner.z + touching_epsilon
            
        }
    };
    
    return overlaps(box1, box2);
}
