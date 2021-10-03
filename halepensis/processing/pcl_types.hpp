#pragma once


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

using point = pcl::PointNormal;
using point_cloud = pcl::PointCloud<point>;
using normal = pcl::Normal;
using surface_normals = pcl::PointCloud<normal>;
using local_features = pcl::PointCloud<pcl::FPFHSignature33>;
using point_indices = pcl::PointIndices;
using model_coefficients = pcl::ModelCoefficients;
