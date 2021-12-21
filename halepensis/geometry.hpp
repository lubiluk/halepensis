#pragma once

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <Eigen/Geometry>
#include <pcl/point_types.h>
#pragma clang diagnostic pop

namespace pcl
{
struct PointNormal;
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
struct PointIndices;
}

using mat44 = Eigen::Matrix<float, 4, 4>;
using point = pcl::PointNormal;
using point_cloud = pcl::PointCloud<point>;
using point_indices = pcl::PointIndices;
using normal = pcl::Normal;
using surface_normals = pcl::PointCloud<normal>;
using vec3 = Eigen::Vector3f;
using rot_mat = Eigen::Matrix3f;
using quat = Eigen::Quaternionf;
using model_coefficients = pcl::ModelCoefficients;

inline auto point_to_vec3(const point& p) -> vec3
{
    return {p.x, p.y, p.z};
}
