#pragma once

#include "pcl_types.hpp"
#include <memory>
#include <tuple>
#include <optional>


auto fit_circle(const std::shared_ptr<point_cloud>& input_cloud,
                const std::shared_ptr<surface_normals>& input_normals)
-> std::optional<std::tuple<std::shared_ptr<point_indices>, std::shared_ptr<model_coefficients>>>;
