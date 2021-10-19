#pragma once

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <Eigen/Geometry>
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

using Transform = Eigen::Matrix<float, 4, 4>;
using Point = pcl::PointNormal;
using PointCloud = pcl::PointCloud<Point>;
