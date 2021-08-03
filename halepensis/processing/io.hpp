#pragma once

#include "pcl_types.hpp"
#include <memory>
#include <string>

auto load_cloud(const std::string& pcd_file) -> std::shared_ptr<point_cloud>;
