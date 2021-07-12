#include "entity_viewing.hpp"
#include "entity.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/visualization/pcl_visualizer.h>
#pragma clang diagnostic pop

std::vector<std::array<int, 3>> color_pallete {
    {255, 89, 94},
    {255, 202, 58},
    {138, 201, 38},
    {25, 130, 196},
    {106, 76, 147},
};

void view_entity(const entity& item)
{
    pcl::visualization::PCLVisualizer viz("3D Viewer");
    viz.setBackgroundColor(0, 0, 0);
    viz.initCameraParameters();
    viz.setCameraPosition(0, 0, 0, 0, -1, 0);
    
    int i = 0;
    auto cloud = item.surface.cloud;
    
    pcl::visualization::PointCloudColorHandlerCustom<point> color(cloud,
                                                                  color_pallete[i % color_pallete.size()][0],
                                                                  color_pallete[i % color_pallete.size()][1],
                                                                  color_pallete[i % color_pallete.size()][2]);
    viz.addPointCloud<point>(cloud, color, "cloud");
    
    viz.spin();
    viz.close();
}
