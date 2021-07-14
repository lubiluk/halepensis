#pragma once

#include "types.hpp"
#include <vector>

class entity_surface;

enum class component_type {
    surface, cylinder
};

class entity_component {
public:
    const component_type type;
    const std::shared_ptr<point_indices> indices;
    const std::shared_ptr<point_cloud> cloud;
    const std::shared_ptr<model_coefficients> coefficients;
    
    entity_component(const component_type type,
                     const std::shared_ptr<point_indices> indices,
                     const std::shared_ptr<point_cloud> cloud,
                     const std::shared_ptr<model_coefficients> coefficients);
};

auto recognize_components(const component_type type, const entity_surface& surface) -> std::vector<const entity_component>;
