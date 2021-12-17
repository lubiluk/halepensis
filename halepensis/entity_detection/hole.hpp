#pragma once

#include "geometry.hpp"
#include <memory>
#include <vector>

auto find_holes(const std::shared_ptr<point_cloud> &cloud)
-> std::vector<std::shared_ptr<point_cloud>>;
