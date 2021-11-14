#include "below.hpp"

const float epsilon_vertical = 0.01;
const float epsilon_horizontal = 0.025;

auto is_below(const Entity& entity1, const Entity& entity2) -> bool
{
    return entity1.position.y() < entity2.position.y() - epsilon_vertical
    && entity1.position.x() < entity2.position.x() + epsilon_horizontal
    && entity1.position.x() > entity2.position.x() - epsilon_horizontal
    && entity1.position.z() < entity2.position.z() + epsilon_horizontal
    && entity1.position.z() > entity2.position.z() - epsilon_horizontal;
}
