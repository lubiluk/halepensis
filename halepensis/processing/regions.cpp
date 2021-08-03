#include "regions.hpp"
#include <algorithm>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
#pragma clang diagnostic pop

auto segment_regions(const std::shared_ptr<point_cloud>& input_cloud,
                     const std::shared_ptr<surface_normals>& input_normals)
-> std::vector<std::shared_ptr<point_indices>>
{
    pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::removeNaNFromPointCloud(*input_cloud, *indices);
    
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (10);
    reg.setMaxClusterSize (1000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (10);
    reg.setInputCloud (input_cloud);
    reg.setIndices (indices);
    reg.setInputNormals (input_normals);
    reg.setSmoothnessThreshold (1.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (1.0);
    
    std::vector<point_indices> clusters;
    reg.extract(clusters);
    
    // extraction method wants pointers hence the conversion
    std::vector<std::shared_ptr<point_indices>> cluster_ptrs;
    std::transform(clusters.begin(), clusters.end(), std::back_inserter(cluster_ptrs), [](const auto& i) -> auto
    {
        return std::make_shared<point_indices>(i);
    });
    
    return cluster_ptrs;
}
