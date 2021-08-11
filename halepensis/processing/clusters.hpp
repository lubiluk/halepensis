#pragma once

#include "pcl_types.hpp"
#include <vector>
#include <memory>

auto extract_euclidean_clusters(const std::shared_ptr<point_cloud>& input,
                      const double tolerance = 0.02,
                      const int min_size = 100,
                      const int max_size = 25000)
-> std::vector<std::shared_ptr<point_indices>>;
