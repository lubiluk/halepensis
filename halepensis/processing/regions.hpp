#pragma once

#include "pcl_types.hpp"
#include <vector>
#include <memory>


auto segment_regions(const std::shared_ptr<point_cloud>& input_cloud)
-> std::vector<std::shared_ptr<point_indices>>;
