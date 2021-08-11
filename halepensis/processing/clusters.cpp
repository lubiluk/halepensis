#include "clusters.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#pragma clang diagnostic pop

auto extract_euclidean_clusters(const std::shared_ptr<point_cloud>& input,
                      const double tolerance,
                      const int min_size,
                      const int max_size)
-> std::vector<std::shared_ptr<point_indices>>
{
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<point>::Ptr tree(new pcl::search::KdTree<point>);
    tree->setInputCloud (input);
    
    std::vector<point_indices> cluster_indices;
    pcl::EuclideanClusterExtraction<point> ec;
    ec.setClusterTolerance(tolerance);
    ec.setMinClusterSize(min_size);
    ec.setMaxClusterSize(max_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input);
    ec.extract(cluster_indices);
    
    // extraction method wants pointers hence the conversion
    std::vector<std::shared_ptr<point_indices>> cluster_ptrs;
    std::transform(cluster_indices.begin(), cluster_indices.end(), std::back_inserter(cluster_ptrs), [](const auto& i) -> auto
    {
        return std::make_shared<point_indices>(i);
    });
    
    return cluster_ptrs;
}
