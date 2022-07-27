#pragma once

#include "geometry.hpp"
#include <memory>
#include <vector>

auto detect_edges(const std::shared_ptr<point_cloud> &cloud)
-> std::vector<std::shared_ptr<point_cloud>>;
