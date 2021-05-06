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
}

using Point = pcl::PointXYZ;
using Normal = pcl::Normal;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using SurfaceNormals = pcl::PointCloud<pcl::Normal>;
using LocalFeatures = pcl::PointCloud<pcl::FPFHSignature33>;
using SearchMethod = pcl::search::KdTree<pcl::PointXYZ>;
using VoxelGrid = pcl::VoxelGrid<pcl::PointXYZ>;
using PassThrough = pcl::PassThrough<pcl::PointXYZ>;
using ExtractIndices = pcl::ExtractIndices<pcl::PointXYZ>;
