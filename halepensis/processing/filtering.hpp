#pragma once

#include "pcl_types.hpp"
#include <memory>
#include <string>

auto extract_cloud(const std::shared_ptr<point_cloud>& input_cloud,
                   const std::shared_ptr<point_indices>& indices,
                   const bool negative = false) -> std::shared_ptr<point_cloud>;

auto extract_normals(const std::shared_ptr<surface_normals>& input_normals,
                     const std::shared_ptr<point_indices>& indices) -> std::shared_ptr<surface_normals>;

auto downsample(const std::shared_ptr<point_cloud>& input,
                const float voxel_grid_size = 0.005f) -> std::shared_ptr<point_cloud>;

auto filter_field(const std::shared_ptr<point_cloud>& input,
                  const std::string& field_name = "z",
                  const double threshold = 1.0,
                  const bool negative = false) -> std::shared_ptr<point_cloud>;
