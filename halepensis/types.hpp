#pragma once

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#pragma clang diagnostic pop

using Point = pcl::PointXYZ;
using Normal = pcl::Normal;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using SurfaceNormals = pcl::PointCloud<pcl::Normal>;
using LocalFeatures = pcl::PointCloud<pcl::FPFHSignature33>;
using SearchMethod = pcl::search::KdTree<pcl::PointXYZ>;
using VoxelGrid = pcl::VoxelGrid<pcl::PointXYZ>;
using PassThrough = pcl::PassThrough<pcl::PointXYZ>;
using ExtractIndices = pcl::ExtractIndices<pcl::PointXYZ>;
