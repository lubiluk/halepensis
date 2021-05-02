#pragma once

#include "types.hpp"
#include "error.hpp"

PointCloud::Ptr downsample(PointCloud::ConstPtr input);
PointCloud::Ptr removeBackground(PointCloud::ConstPtr input,
                                 double threshold = 1.0);
PointCloud::Ptr loadCloud(const std::string pcd_file) throw(Error);