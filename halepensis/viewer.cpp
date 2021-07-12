#include "viewer.hpp"

#include <thread>

using namespace std::chrono_literals;

std::vector<std::array<int, 3>> Viewer::color_pallete {
    {255, 89, 94},
    {255, 202, 58},
    {138, 201, 38},
    {25, 130, 196},
    {106, 76, 147},
};

void Viewer::view(const PointCloud::ConstPtr cloud)
{
    view(std::vector<PointCloud::ConstPtr> {cloud});
}

void Viewer::view(const std::initializer_list<PointCloud::ConstPtr> &clouds_init, bool split_viewports)
{
    const std::vector<PointCloud::ConstPtr> clouds(clouds_init);
    view(clouds, split_viewports);
}

void Viewer::view(const std::vector<PointCloud::ConstPtr> & clouds, bool split_viewports)
{
    pcl::visualization::PCLVisualizer viz("3D Viewer" + std::to_string(clouds.size()));
    viz.setBackgroundColor(0, 0, 0);
    viz.initCameraParameters();
    viz.setCameraPosition(0, 0, 0, 0, -1, 0);
    
    for (int i = 0; i < clouds.size(); ++i) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(clouds[i],
                                                                              color_pallete[i % color_pallete.size()][0],
                                                                              color_pallete[i % color_pallete.size()][1],
                                                                              color_pallete[i % color_pallete.size()][2]);
         
        if (split_viewports)
        {
            int viewport(0);
            viz.createViewPort(1.0 / clouds.size() * i, 0.0, 1.0 / clouds.size() * (i + 1), 1.0, viewport);
            viz.addPointCloud<pcl::PointXYZ>(clouds[i], color, "cloud" + std::to_string(i), viewport);
        }
        else
        {
            viz.addPointCloud<pcl::PointXYZ>(clouds[i], color, "cloud" + std::to_string(i));
        }
        
    }

    viz.spin();
    viz.close();
}
