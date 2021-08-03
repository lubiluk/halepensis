#pragma once

#include "pcl_types.hpp"
#include <memory>

auto compute_normals(const std::shared_ptr<point_cloud> &cloud,
                     const float normal_radius = 0.02f) -> std::shared_ptr<surface_normals>;
