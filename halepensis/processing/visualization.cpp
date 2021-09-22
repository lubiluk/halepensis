#include "visualization.hpp"

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

auto view(const std::shared_ptr<point_cloud>& cloud) -> void
{
    pcl::visualization::PCLVisualizer viz("3D Viewer");
    viz.setBackgroundColor(0, 0, 0);
    viz.initCameraParameters();
    viz.setCameraPosition(0, 0, 0, 0, -1, 0);
    
    viz.addPointCloud<point>(cloud, "cloud");
    viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
    
    viz.spin();
    viz.close();
}

auto view_clusters(const std::shared_ptr<point_cloud>& cloud,
                   const std::vector<std::shared_ptr<point_cloud>>& clusters) -> void
{
    pcl::visualization::PCLVisualizer viz("3D Viewer");
    viz.setBackgroundColor(0, 0, 0);
    viz.initCameraParameters();
    viz.setCameraPosition(0, 0, 0, 0, -1, 0);
    
    viz.addPointCloud<point>(cloud, "cloud");
    viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    
    for(std::size_t i = 0; i < clusters.size(); ++i) {
        const auto& cloud = clusters[i];
        pcl::visualization::PointCloudColorHandlerCustom<point>
        color(cloud,
              color_pallete[i % color_pallete.size()][0],
              color_pallete[i % color_pallete.size()][1],
              color_pallete[i % color_pallete.size()][2]);
        
        viz.addPointCloud<point>(cloud, color, "cluster_" + std::to_string(i));
        viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cluster_" + std::to_string(i));
    }
    
    
    viz.spin();
    viz.close();
}
