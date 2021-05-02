
#include "utils.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/io/pcd_io.h>
#pragma clang diagnostic pop

PointCloud::Ptr downsample(PointCloud::ConstPtr input)
{
    const float voxel_grid_size = 0.005f;
    VoxelGrid vox_grid;
    PointCloud::Ptr output(new PointCloud);
    vox_grid.setInputCloud(input);
    vox_grid.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
    vox_grid.filter(*output);

    return output;
}

PointCloud::Ptr removeBackground(PointCloud::ConstPtr input,
                                 double threshold)
{
    const float depth_limit = 1.0;
    pcl::PassThrough<pcl::PointXYZ> pass;
    PointCloud::Ptr output(new PointCloud);
    pass.setInputCloud(input);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, depth_limit);
    pass.filter(*output);

    return output;
}

PointCloud::Ptr loadCloud(const std::string pcd_file) throw(Error)
{
    PointCloud::Ptr cloud(new PointCloud);

    if (pcl::io::loadPCDFile<Point>(pcd_file, *cloud) == -1)
    {
        throw Error(std::string("Couldn't read file ") + pcd_file);
    }

    return cloud;
}
