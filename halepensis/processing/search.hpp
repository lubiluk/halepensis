#pragma once

#include "pcl_types.hpp"
#include <vector>
#include <memory>
#include <optional>

auto count_nearby_points(const std::shared_ptr<point_cloud>& input_cloud,
                         const point search_point, const float radius) -> int;

auto point_from_coefficients(std::shared_ptr<model_coefficients> coefficients) -> point;
