#pragma once

#include "pcl_types.hpp"
#include <memory>
#include <vector>

auto find_holes(std::shared_ptr<point_cloud> cloud, std::shared_ptr<surface_normals> normals)
-> std::vector<std::shared_ptr<point_cloud>>;
