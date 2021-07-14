#include "scene_entity.hpp"
#include "types.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#pragma clang diagnostic pop

std::shared_ptr<point_cloud> downsample(const std::shared_ptr<point_cloud>& input)
{
    const float voxel_grid_size = 0.005f;
    voxel_grid vox_grid;
    auto output = std::make_shared<point_cloud>();
    vox_grid.setInputCloud(input);
    vox_grid.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
    vox_grid.filter(*output);

    return output;
}

std::shared_ptr<point_cloud> remove_background(const std::shared_ptr<point_cloud>& input,
                                               double threshold = 1.0)
{
    const float depth_limit = 1.0;
    pcl::PassThrough<point> pass;
    auto output = std::make_shared<point_cloud>();
    pass.setInputCloud(input);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, depth_limit);
    pass.filter(*output);

    return output;
}

std::shared_ptr<point_cloud> load_cloud(const std::string& pcd_file) throw(error)
{
    auto cloud = std::make_shared<point_cloud>();

    if (pcl::io::loadPCDFile<point>(pcd_file, *cloud) == -1)
    {
        throw error(std::string("Couldn't read file ") + pcd_file);
    }

    return cloud;
}

std::vector<entity_component> recognize_components(const entity_surface surface)
{
    std::vector<entity_component> components;
    
    return components;
}

scene_entity::scene_entity(const std::string& pcd_file) throw(error):
surface(downsample(remove_background(load_cloud(pcd_file)))),
components(recognize_components(surface))
{
   
}
