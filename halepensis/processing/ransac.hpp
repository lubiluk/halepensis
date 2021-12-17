#pragma once

#include "geometry.hpp"
#include <memory>
#include <tuple>
#include <optional>
#include <boost/optional.hpp>


auto fit_cylinder(const std::shared_ptr<point_cloud>& input_cloud,
                  const std::shared_ptr<surface_normals>& input_normals)
-> boost::optional<std::tuple<std::shared_ptr<point_indices>, std::shared_ptr<model_coefficients>>>;

auto fit_plane(const std::shared_ptr<point_cloud>& input_cloud)
-> boost::optional<std::tuple<std::shared_ptr<point_indices>, std::shared_ptr<model_coefficients>>>;

auto fit_sphere(const std::shared_ptr<point_cloud>& input_cloud)
-> boost::optional<std::tuple<std::shared_ptr<point_indices>, std::shared_ptr<model_coefficients>>>;
