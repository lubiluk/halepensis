#pragma once

#include "geometry.hpp"
#include <memory>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <Eigen/Geometry>
#pragma clang diagnostic pop

auto align(const std::shared_ptr<point_cloud> &cloud1, const std::shared_ptr<point_cloud> &cloud2)
-> std::pair<Eigen::Matrix<float, 4, 4>, std::shared_ptr<point_cloud>>;
