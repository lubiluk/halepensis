#pragma once

#include "geometry.hpp"
#include "scene_entity.hpp"

auto is_below(const scene_entity& entity1, const scene_entity& entity2) -> bool;
auto is_inside(const scene_entity& entity1, const scene_entity& entity2) -> bool;
