#pragma once

#include "geometry.hpp"
#include <memory>

auto scale(const std::shared_ptr<point_cloud>& cloud, const float factor)
-> std::shared_ptr<point_cloud>;
