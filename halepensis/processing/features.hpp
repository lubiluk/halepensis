#pragma once

#include "pcl_types.hpp"
#include <memory>
#include <vector>

auto estimate_boundaries(const std::shared_ptr<point_cloud>& cloud, const std::shared_ptr<surface_normals>& normals)
-> std::shared_ptr<point_indices>;
