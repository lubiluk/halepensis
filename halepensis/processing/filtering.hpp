#pragma once

#include "pcl_types.hpp"
#include <memory>

auto extract_cloud(const std::shared_ptr<point_cloud>& input_cloud,
                   const std::shared_ptr<point_indices>& indices,
                   const bool negative = false) -> std::shared_ptr<point_cloud>;

auto extract_normals(const std::shared_ptr<surface_normals>& input_normals,
                     const std::shared_ptr<point_indices>& indices) -> std::shared_ptr<surface_normals>;

auto downsample(const std::shared_ptr<point_cloud>& input,
                const float voxel_grid_size = 0.005f) -> std::shared_ptr<point_cloud>;

auto filter_depth(const std::shared_ptr<point_cloud>& input,
                       const double threshold = 1.0) -> std::shared_ptr<point_cloud>;
