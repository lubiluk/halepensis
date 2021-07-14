#pragma once

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/search/kdtree.h>
#pragma clang diagnostic pop


namespace pcl
{
struct PointXYZ;
struct Normal;
struct FPFHSignature33;
template<class T>
class PointCloud;
template<class T>
class VoxelGrid;
template<class T>
class PassThrough;
template<class T>
class ExtractIndices;
struct ModelCoefficients;
}

using point = pcl::PointXYZ;
using point_cloud = pcl::PointCloud<point>;
using normal = pcl::Normal;
using sufrace_normals = pcl::PointCloud<normal>;
using local_features = pcl::PointCloud<pcl::FPFHSignature33>;
using search_method = pcl::search::KdTree<point>;
using point_indices = pcl::PointIndices;
using model_coefficients = pcl::ModelCoefficients;
using voxel_grid = pcl::VoxelGrid<point>;
