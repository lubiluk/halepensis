#pragma once

#include "pcl_types.hpp"
#include <vector>

auto view(const std::shared_ptr<point_cloud>& cloud) -> void;
auto view_clusters(const std::shared_ptr<point_cloud>& cloud,
                   const std::vector<std::shared_ptr<point_cloud>>& clusters) -> void;
