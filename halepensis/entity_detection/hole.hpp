#pragma once

#include "geometry.hpp"
#include <memory>
#include <vector>

auto detect_holes(const std::shared_ptr<point_cloud> &cloud, float pixel_radius = 0.005)
-> std::vector<std::shared_ptr<point_cloud>>;
