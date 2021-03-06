#pragma once

#include "geometry.hpp"
#include <memory>
#include <tuple>
#include <optional>
#include <boost/optional.hpp>


auto fit_cylinder(const std::shared_ptr<point_cloud>& input_cloud)
-> boost::optional<std::tuple<std::shared_ptr<point_indices>, std::shared_ptr<model_coefficients>>>;

auto fit_plane(const std::shared_ptr<point_cloud>& input_cloud, float distance_threshold = 0.01)
-> boost::optional<std::tuple<std::shared_ptr<point_indices>, std::shared_ptr<model_coefficients>>>;

auto fit_sphere(const std::shared_ptr<point_cloud>& input_cloud)
-> boost::optional<std::tuple<std::shared_ptr<point_indices>, std::shared_ptr<model_coefficients>>>;

auto fit_circle(const std::shared_ptr<point_cloud>& input_cloud)
-> boost::optional<std::tuple<std::shared_ptr<point_indices>, std::shared_ptr<model_coefficients>>>;
