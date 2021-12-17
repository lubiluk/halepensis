#pragma once

#include "geometry.hpp"
#include <memory>

auto filter_outliers(const std::shared_ptr<point_cloud>& input_cloud) -> std::shared_ptr<point_cloud>;
