#include "entity_component.hpp"

entity_component::entity_component(component_type type,
                     std::shared_ptr<point_indices> indices,
                     std::shared_ptr<point_cloud> cloud,
                     std::shared_ptr<model_coefficients> coefficients):
    type(type), indices(indices), cloud(cloud), coefficients(coefficients)
{
    
}
