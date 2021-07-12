#pragma once

#include "types.hpp"

class entity_surface
{
public:
    const std::shared_ptr<point_cloud> cloud;
    const std::shared_ptr<search_method> method;
    const float normal_radius;
    const float feature_radius;
    const std::shared_ptr<sufrace_normals> normals;
    const std::shared_ptr<local_features> features;
    
    entity_surface(std::shared_ptr<point_cloud> cloud);
};

