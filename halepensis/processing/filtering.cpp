#include "filtering.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#pragma clang diagnostic pop


auto extract_cloud(const std::shared_ptr<point_cloud>& input_cloud,
                   const std::shared_ptr<point_indices>& indices,
                   const bool negative) -> std::shared_ptr<point_cloud>
{
    pcl::ExtractIndices<point> extract;
    extract.setInputCloud(input_cloud);
    
    const auto result_cloud = std::make_shared<point_cloud>();
    
    extract.setIndices(indices);
    extract.setNegative(negative);
    extract.filter(*result_cloud);
    
    return result_cloud;
}

auto extract_normals(const std::shared_ptr<surface_normals>& input_normals,
                   const std::shared_ptr<point_indices>& indices) -> std::shared_ptr<surface_normals>
{
    pcl::ExtractIndices<normal> extract;
    extract.setInputCloud(input_normals);
    
    const auto result_normals = std::make_shared<surface_normals>();
    
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*result_normals);
    
    return result_normals;
}

auto downsample(const std::shared_ptr<point_cloud>& input,
                const float voxel_grid_size) -> std::shared_ptr<point_cloud>
{
    pcl::VoxelGrid<point> vox_grid;
    auto output = std::make_shared<point_cloud>();
    vox_grid.setInputCloud(input);
    vox_grid.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
    vox_grid.filter(*output);
    
    return output;
}

auto filter_depth(const std::shared_ptr<point_cloud>& input,
                       const double threshold) -> std::shared_ptr<point_cloud>
{
    pcl::PassThrough<point> pass;
    auto output = std::make_shared<point_cloud>();
    pass.setInputCloud(input);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, threshold);
    pass.filter(*output);
    
    return output;
}
