#pragma once

#include "entity_surface.hpp"
#include "error.hpp"
#include <string>
#include <vector>

enum class component_type;
class entity_component;

class scene_entity
{
public:
    const entity_surface surface;
    
    scene_entity(const std::string& pcd_file) throw(error);
    auto components(const component_type type) -> std::vector<const entity_component>;
};
