#pragma once

#include "geometry.hpp"
#include <memory>
#include <vector>

auto detect_pegs(const std::shared_ptr<PointCloud> &cloud)
-> std::vector<std::shared_ptr<PointCloud>>;
