#pragma once

#include "pcl_types.hpp"
#include <memory>
#include <vector>

auto detect_objects(const std::shared_ptr<point_cloud> &cloud, const int n = 2)
-> std::vector<std::shared_ptr<point_cloud>>;
