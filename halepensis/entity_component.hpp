#pragma once

#include "types.hpp"

enum class component_type {
    surface, cylinder
};

class entity_component {
public:
    const component_type type;
    const std::shared_ptr<point_indices> indices;
    const std::shared_ptr<point_cloud> cloud;
    const std::shared_ptr<model_coefficients> coefficients;
    
    entity_component(component_type type,
              std::shared_ptr<point_indices> indices,
              std::shared_ptr<point_cloud> cloud,
              std::shared_ptr<model_coefficients> coefficients);
};
