#include "viewer.hpp"

#include <thread>

using namespace std::chrono_literals;

void Viewer::view(const Object &object)
{
    Viewer::view(object.getPointCloud());
}

void Viewer::view(const Scene &scene)
{
    Viewer::view(scene.getPointCloud());
}

void Viewer::view(const PointCloud::Ptr cloud) {
    pcl::visualization::PCLVisualizer viz("3D Viewer");
    viz.setBackgroundColor(0, 0, 0);
    viz.initCameraParameters();
    viz.setCameraPosition(0, 0, 0, 0, -1, 0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud, 1, 0, 0);
    viz.addPointCloud<pcl::PointXYZ>(cloud, "cloud");

    while (!viz.wasStopped())
    {
        viz.spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
}