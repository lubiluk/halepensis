#pragma once

#include "entity_surface.hpp"
#include "entity_component.hpp"
#include "error.hpp"
#include <string>
#include <vector>

class scene_entity
{
public:
    const entity_surface surface;
    const std::vector<entity_component> components;
    
    scene_entity(const std::string& pcd_file) throw(error);
};
